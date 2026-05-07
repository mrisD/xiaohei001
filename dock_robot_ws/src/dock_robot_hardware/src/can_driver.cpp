#include "can_driver.hpp"
#include <iostream>
#include <chrono>

CAN_DRIVER::CAN_DRIVER()
{
    std::cout << "CAN_DRIVER initialized" << std::endl;
}

CAN_DRIVER::~CAN_DRIVER()
{
    can_close();
}

int32_t CAN_DRIVER::angle_to_cnt(float angle)
{
    return (int32_t)(angle * ENCODER_CNT * REDUCTION_RATIO / 360.0f);
}

float CAN_DRIVER::cnt_to_angle(int32_t cnt)
{
    return (float)cnt * 360.0f / (ENCODER_CNT * REDUCTION_RATIO);
}

bool CAN_DRIVER::can_open()
{
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK) return false;

    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 0;
    config.Timing0 = 0x00;  // 500kbps
    config.Timing1 = 0x14;
    config.Mode = 0;

    if (VCI_InitCAN(VCI_USBCAN2, 0, CAN_CHANNEL, &config) != STATUS_OK) return false;
    if (VCI_StartCAN(VCI_USBCAN2, 0, CAN_CHANNEL) != STATUS_OK) return false;

    // 启动接收线程
    is_running_ = true;
    receive_thread_ = std::thread(&CAN_DRIVER::receive_thread_func, this);
    
    
    is_canOpened_ = true;
    // update_states();
    printf("CAN初始化成功，启动接收线程\n");
    return true;
}

bool CAN_DRIVER::can_close()
{
    if (!is_canOpened_) {
        return true;
    }
    
    is_running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    VCI_CloseDevice(VCI_USBCAN2, 0);
    is_canOpened_ = false;
    printf("CAN设备关闭\n");
    return true;
}

// 接收线程：持续读取CAN数据，不阻塞主线程
void CAN_DRIVER::receive_thread_func()
{
    printf("CAN接收线程启动\n");
    auto last_query_time = std::chrono::steady_clock::now();
    while (is_running_) {
        // 非阻塞接收，等待时间1ms
        ULONG len = VCI_Receive(VCI_USBCAN2, 0, CAN_CHANNEL, recv_buffers, 100, 1);
        
        // printf("CAN接收中 %d\n",len);
        if (len > 0) {
            for (ULONG i = 0; i < len; i++) {
                parse_can_message(recv_buffers[i]);
            }
        }

        // ===================== 自动定时请求电机位置 =====================
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_query_time).count() >= 20) {
            update_states(); // 每20ms主动读一次所有电机
            last_query_time = now;
        }

        
        // 发送待处理的命令
        send_pending_commands();
        
        // 小延时避免CPU占用过高
        // std::this_thread::sleep_for(std::chrono::microseconds(500));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// 解析CAN消息并更新电机状态
void CAN_DRIVER::parse_can_message(const VCI_CAN_OBJ& obj)
{
    // 根据你的电机协议解析
    // 这里假设数据格式：Data[0]是命令字，Data[1-4]是位置数据
    // uint8_t cmd = obj.Data[0];
    // printf("收到电机ID=0x%02X 命令=0x%02X\n", obj.ID, cmd); //
    if (obj.DataLen >= 5 && obj.Data[0] == 0x08) {
        int32_t cnt;
        memcpy(&cnt, &obj.Data[1], 4);
        float angle = cnt_to_angle(cnt);
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_states_[obj.ID].position_deg = angle;
        motor_states_[obj.ID].timestamp = obj.TimeStamp;
        motor_states_[obj.ID].updated = true;
    }
    // 可以添加其他命令字的解析
}

// 发送命令（非阻塞，只放入队列）
bool CAN_DRIVER::motor_set_angle(unsigned int motor_id, float angle)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    pending_commands_[motor_id] = angle;
    return true;
}

// 批量发送多个电机命令
bool CAN_DRIVER::motor_set_angle_batch(const std::map<unsigned int, float>& commands)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (const auto& cmd : commands) {
        pending_commands_[cmd.first] = cmd.second;
    }
    return true;
}

// 发送待处理的命令（在接收线程中调用）
void CAN_DRIVER::send_pending_commands()
{
    std::map<unsigned int, float> commands_to_send;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        commands_to_send.swap(pending_commands_);
    }
    
    for (const auto& cmd : commands_to_send) {
        int32_t cnt = angle_to_cnt(cmd.second);
        
        memset(&send_obj, 0, sizeof(send_obj));
        send_obj.ID = cmd.first;
        send_obj.SendType = 1;
        send_obj.RemoteFlag = 0;
        send_obj.ExternFlag = 0;
        send_obj.DataLen = 5;
        send_obj.Data[0] = 0x1E;
        memcpy(&send_obj.Data[1], &cnt, 4);
        
        // 立即发送，不等待回复
        VCI_Transmit(VCI_USBCAN2, 0, CAN_CHANNEL, &send_obj, 1);
    }
}

// 获取最新角度（非阻塞）
float CAN_DRIVER::motor_get_angle(unsigned int motor_id)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (motor_states_.find(motor_id) != motor_states_.end()) {
        return motor_states_[motor_id].position_deg;
    }
    return 0.0;
}

// 获取完整电机状态
MotorState CAN_DRIVER::motor_get_state(unsigned int motor_id)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (motor_states_.find(motor_id) != motor_states_.end()) {
        return motor_states_[motor_id];
    }
    return MotorState();
}

// 主动请求更新所有电机状态（如果需要）
void CAN_DRIVER::update_states()
{
    // 发送请求命令读取所有电机
    // 这样电机才会主动发送当前状态
    for (int id : {RIGHT_MOTOR1, RIGHT_MOTOR2, RIGHT_MOTOR3, RIGHT_MOTOR4, RIGHT_MOTOR5}) {
        memset(&send_obj, 0, sizeof(send_obj));
        send_obj.ID = id;
        send_obj.SendType = 1;
        send_obj.RemoteFlag = 0;
        send_obj.ExternFlag = 0;
        send_obj.DataLen = 1;
        send_obj.Data[0] = 0x08;  // 读取命令
        
        VCI_Transmit(VCI_USBCAN2, 0, CAN_CHANNEL, &send_obj, 1);
    }
}