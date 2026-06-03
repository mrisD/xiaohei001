#include "can_driver.hpp"

#include <iostream>

// ==========================
// 构造 / 析构
// ==========================
CAN_DRIVER::CAN_DRIVER()
{
    std::cout << "CAN_DRIVER initialized" << std::endl;
}

CAN_DRIVER::~CAN_DRIVER()
{
    can_close();
}

// ==========================
// 角度 -> 编码器计数
// ==========================
int32_t CAN_DRIVER::angle_to_cnt(float angle)
{
    return static_cast<int32_t>(
        angle * ENCODER_CNT * REDUCTION_RATIO / 360.0f
    );
}

// ==========================
// 编码器计数 -> 角度
// ==========================
float CAN_DRIVER::cnt_to_angle(int32_t cnt)
{
    return static_cast<float>(cnt) * 360.0f /
           static_cast<float>(ENCODER_CNT * REDUCTION_RATIO);
}

// ==========================
// 根据电机 ID 判断 CAN 通道
// ==========================
int CAN_DRIVER::get_motor_channel(unsigned int motor_id) const
{
    switch (motor_id) {
        case LEFT_MOTOR1:
        case LEFT_MOTOR2:
        case LEFT_MOTOR3:
        case LEFT_MOTOR4:
        case LEFT_MOTOR5:
            return CAN_CHANNEL_LEFT;

        case RIGHT_MOTOR1:
        case RIGHT_MOTOR2:
        case RIGHT_MOTOR3:
        case RIGHT_MOTOR4:
        case RIGHT_MOTOR5:
            return CAN_CHANNEL_RIGHT;

        default:
            printf("警告：未知 motor_id=0x%X，默认使用 CAN_CHANNEL_RIGHT\n", motor_id);
            return CAN_CHANNEL_RIGHT;
    }
}

// ==========================
// 打开 CAN 设备，同时初始化 CAN1 和 CAN2
// ==========================
bool CAN_DRIVER::can_open()
{
    if (is_canOpened_.load()) {
        printf("CAN设备已经打开\n");
        return true;
    }

    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK) {
        printf("VCI_OpenDevice 失败\n");
        return false;
    }

    memset(&config, 0, sizeof(config));

    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 0;

    // 500kbps
    config.Timing0 = 0x00;
    config.Timing1 = 0x14;

    // 正常模式
    config.Mode = 0;

    // ==========================
    // 初始化 CAN1，左臂
    // ==========================
    if (VCI_InitCAN(VCI_USBCAN2, 0, CAN_CHANNEL_LEFT, &config) != STATUS_OK) {
        printf("CAN_CHANNEL_LEFT 初始化失败\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return false;
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, CAN_CHANNEL_LEFT) != STATUS_OK) {
        printf("CAN_CHANNEL_LEFT 启动失败\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return false;
    }

    // ==========================
    // 初始化 CAN2，右臂
    // ==========================
    if (VCI_InitCAN(VCI_USBCAN2, 0, CAN_CHANNEL_RIGHT, &config) != STATUS_OK) {
        printf("CAN_CHANNEL_RIGHT 初始化失败\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return false;
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, CAN_CHANNEL_RIGHT) != STATUS_OK) {
        printf("CAN_CHANNEL_RIGHT 启动失败\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return false;
    }

    is_running_.store(true);
    receive_thread_ = std::thread(&CAN_DRIVER::receive_thread_func, this);

    is_canOpened_.store(true);

    printf("CAN初始化成功：CAN1=左臂，CAN2=右臂，接收线程已启动\n");
    return true;
}

// ==========================
// 关闭 CAN 设备
// ==========================
bool CAN_DRIVER::can_close()
{
    if (!is_canOpened_.load()) {
        return true;
    }

    is_running_.store(false);

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    VCI_CloseDevice(VCI_USBCAN2, 0);

    is_canOpened_.store(false);

    printf("CAN设备关闭\n");
    return true;
}

// ==========================
// 接收线程：同时读取 CAN1 和 CAN2
// ==========================
void CAN_DRIVER::receive_thread_func()
{
    printf("CAN接收线程启动\n");

    auto last_query_time = std::chrono::steady_clock::now();

    while (is_running_.load()) {
        // ==========================
        // 接收 CAN1：左臂
        // ==========================
        ULONG len_left = VCI_Receive(
            VCI_USBCAN2,
            0,
            CAN_CHANNEL_LEFT,
            recv_buffers_,
            100,
            1
        );

        if (len_left > 0) {
            for (ULONG i = 0; i < len_left; ++i) {
                parse_can_message(recv_buffers_[i]);
            }
        }

        // ==========================
        // 接收 CAN2：右臂
        // ==========================
        ULONG len_right = VCI_Receive(
            VCI_USBCAN2,
            0,
            CAN_CHANNEL_RIGHT,
            recv_buffers_,
            100,
            1
        );

        if (len_right > 0) {
            for (ULONG i = 0; i < len_right; ++i) {
                parse_can_message(recv_buffers_[i]);
            }
        }

        // ==========================
        // 定时主动查询所有电机位置
        // ==========================
        auto now = std::chrono::steady_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_query_time
            ) >= query_interval_) {
            update_states();
            last_query_time = now;
        }

        // ==========================
        // 发送待处理命令
        // ==========================
        send_pending_commands();

        // 降低 CPU 占用
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    printf("CAN接收线程退出\n");
}

// ==========================
// 解析 CAN 返回数据
// ==========================
void CAN_DRIVER::parse_can_message(const VCI_CAN_OBJ& obj)
{
    // 当前协议假设：
    // Data[0] == 0x08 表示位置返回
    // Data[1] ~ Data[4] 是 int32_t 位置计数

    if (obj.DataLen >= 5 && obj.Data[0] == 0x08) {
        int32_t cnt = 0;
        memcpy(&cnt, &obj.Data[1], 4);

        float angle = cnt_to_angle(cnt);

        std::lock_guard<std::mutex> lock(state_mutex_);

        motor_states_[obj.ID].position_deg = angle;
        motor_states_[obj.ID].timestamp = obj.TimeStamp;
        motor_states_[obj.ID].updated = true;

        // 调试时可以打开
        // printf("收到位置：ID=0x%X angle=%.3f deg cnt=%d\n",
        //        obj.ID, angle, cnt);
    }
}

// ==========================
// 设置单个电机角度
// 非阻塞，只写入命令缓存
// ==========================
bool CAN_DRIVER::motor_set_angle(unsigned int motor_id, float angle)
{
    if (!is_canOpened_.load()) {
        printf("motor_set_angle 失败：CAN未打开\n");
        return false;
    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);

    pending_commands_[motor_id] = angle;

    return true;
}

// ==========================
// 批量设置多个电机角度
// 非阻塞，只写入命令缓存
// ==========================
bool CAN_DRIVER::motor_set_angle_batch(
    const std::map<unsigned int, float>& commands
)
{
    if (!is_canOpened_.load()) {
        printf("motor_set_angle_batch 失败：CAN未打开\n");
        return false;
    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);

    for (const auto& cmd : commands) {
        pending_commands_[cmd.first] = cmd.second;
    }

    return true;
}

// ==========================
// 发送缓存中的待处理命令
// 根据 motor_id 自动选择 CAN1 或 CAN2
// ==========================
void CAN_DRIVER::send_pending_commands()
{
    std::map<unsigned int, float> commands_to_send;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        commands_to_send.swap(pending_commands_);
    }

    for (const auto& cmd : commands_to_send) {
        unsigned int motor_id = cmd.first;
        float target_angle = cmd.second;

        int channel = get_motor_channel(motor_id);
        int32_t cnt = angle_to_cnt(target_angle);

        VCI_CAN_OBJ send_obj;
        memset(&send_obj, 0, sizeof(send_obj));

        send_obj.ID = motor_id;
        send_obj.SendType = 1;
        send_obj.RemoteFlag = 0;
        send_obj.ExternFlag = 0;
        send_obj.DataLen = 5;

        // 0x1E：设置目标角度命令
        send_obj.Data[0] = 0x1E;
        memcpy(&send_obj.Data[1], &cnt, 4);

        ULONG ret = VCI_Transmit(
            VCI_USBCAN2,
            0,
            channel,
            &send_obj,
            1
        );

        if (ret != 1) {
            printf("发送失败：motor_id=0x%X channel=%d angle=%.3f cnt=%d ret=%u\n",
                   motor_id,
                   channel,
                   target_angle,
                   cnt,
                   ret);
        } else {
            // 调试时可以打开
            // printf("发送成功：motor_id=0x%X channel=%d angle=%.3f cnt=%d\n",
            //        motor_id, channel, target_angle, cnt);
        }
    }
}

// ==========================
// 获取单个电机最新角度
// ==========================
float CAN_DRIVER::motor_get_angle(unsigned int motor_id)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    auto it = motor_states_.find(motor_id);

    if (it != motor_states_.end()) {
        return it->second.position_deg;
    }

    return 0.0f;
}

// ==========================
// 获取单个电机完整状态
// ==========================
MotorState CAN_DRIVER::motor_get_state(unsigned int motor_id)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    auto it = motor_states_.find(motor_id);

    if (it != motor_states_.end()) {
        return it->second;
    }

    return MotorState();
}

// ==========================
// 主动请求所有电机状态
// 左臂走 CAN1，右臂走 CAN2
// ==========================
void CAN_DRIVER::update_states()
{
    if (!is_canOpened_.load()) {
        return;
    }

    const unsigned int motor_ids[] = {
        LEFT_MOTOR1,
        LEFT_MOTOR2,
        LEFT_MOTOR3,
        LEFT_MOTOR4,
        LEFT_MOTOR5,

        RIGHT_MOTOR1,
        RIGHT_MOTOR2,
        RIGHT_MOTOR3,
        RIGHT_MOTOR4,
        RIGHT_MOTOR5
    };

    for (unsigned int id : motor_ids) {
        int channel = get_motor_channel(id);

        VCI_CAN_OBJ send_obj;
        memset(&send_obj, 0, sizeof(send_obj));

        send_obj.ID = id;
        send_obj.SendType = 1;
        send_obj.RemoteFlag = 0;
        send_obj.ExternFlag = 0;
        send_obj.DataLen = 1;

        // 0x08：读取当前位置
        send_obj.Data[0] = 0x08;

        ULONG ret = VCI_Transmit(
            VCI_USBCAN2,
            0,
            channel,
            &send_obj,
            1
        );

        if (ret != 1) {
            printf("查询失败：motor_id=0x%X channel=%d ret=%u\n",
                   id,
                   channel,
                   ret);
        }
    }
}