#include "can_driver.hpp"
#include <iostream>
#include <fstream>   // 文件操作必备头文件
CAN_DRIVER::CAN_DRIVER()
{
    std::cout<<"i am from testcan +++++++++++"<<std::endl;
    printf("i am from testcan2 +++++++++++");

    // 1. 创建/打开文件（如果文件不存在会自动创建）
    // ios::out = 写入模式；ios::trunc = 覆盖原有内容
    std::ofstream outFile("test_data.txt", std::ios::out | std::ios::trunc);

    // 判断文件是否成功打开
    if (!outFile.is_open()) {
        std::cout << "文件打开失败！" << std::endl;
       // return -1;
    }

    // 2. 写入数据（可以写字符串、数字、变量）
    outFile << "===== 测试数据 =====" << std::endl;
    outFile << "Hello ROS2 & CAN" << std::endl;
    outFile << "当前角度：35.5°" << std::endl;
    outFile << "编码器值：123456" << std::endl;

    // 写入变量示例
    float angle = 90.0f;
    int encoder = 65535;
    outFile << "角度变量：" << angle << std::endl;
    outFile << "编码器变量：" << encoder << std::endl;

    // 3. 关闭文件（好习惯）
    outFile.close();
    std::cout << "数据写入完成！文件已保存为 test_data.txt" << std::endl;
}

CAN_DRIVER::~CAN_DRIVER()
{

}


// 角度 → 编码器计数值
int32_t CAN_DRIVER::angle_to_cnt(float angle)
{
    return (int32_t)(angle * ENCODER_CNT * REDUCTION_RATIO / 360.0f);
}

// 编码器计数值 → 角度
float CAN_DRIVER::cnt_to_angle(int32_t cnt)
{
    return (float)cnt * 360.0f / (ENCODER_CNT * REDUCTION_RATIO);
}

// 打开 CAN 设备
bool CAN_DRIVER::can_open()
{
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK) return false;

    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 0;
    config.Timing0 = 0x00;
    config.Timing1 = 0x14;
    config.Mode = 0;

    if (VCI_InitCAN(VCI_USBCAN2, 0, CAN_CHANNEL, &config) != STATUS_OK) return false;
    if (VCI_StartCAN(VCI_USBCAN2, 0, CAN_CHANNEL) != STATUS_OK) return false;

    printf("CAN初始化成功\n");
    is_canOpened=true;
    return is_canOpened;
}

bool CAN_DRIVER::can_close()
{
    if (!is_canOpened) {
        printf("CAN设备未打开，无需关闭\n");
        return true;
    }
    
    printf("正在关闭CAN设备...\n");
    
    // 停止CAN通信
    // if (VCI_StopCAN(VCI_USBCAN2, 0, CAN_CHANNEL) != STATUS_OK) {
    //     printf("警告：停止CAN失败\n");
    // }
    
    // 关闭设备
    int ret = VCI_CloseDevice(VCI_USBCAN2, 0);
    
    if (ret == STATUS_OK) {
        is_canOpened = false;
        printf("CAN设备关闭成功\n");
        return true;
    } else {
        printf("CAN设备关闭失败，错误码: %d\n", ret);
        return false;
    }
}

// 电机设置角度
bool CAN_DRIVER::motor_set_angle(unsigned int motor_id,float angle)
{
    int32_t cnt = angle_to_cnt(angle);

    memset(&send_obj, 0, sizeof(send_obj));

    send_obj.ID = motor_id;
    send_obj.SendType = 1;
    send_obj.RemoteFlag = 0;
    send_obj.ExternFlag = 0;
    send_obj.DataLen = 5;

    send_obj.Data[0] = 0x1E;
    memcpy(&send_obj.Data[1], &cnt, 4);

    ULONG ret = VCI_Transmit(VCI_USBCAN2, 0, CAN_CHANNEL, &send_obj, 1);

    printf("发送角度: %.2f° (cnt=%d)\n", angle, cnt);

    return (ret == 1);
}

// 读取电机当前角度
float CAN_DRIVER::motor_read_angle(unsigned int motor_id)
{
    memset(&send_obj, 0, sizeof(send_obj));

    send_obj.ID = motor_id;
    send_obj.SendType = 1;
    send_obj.RemoteFlag = 0;
    send_obj.ExternFlag = 0;
    send_obj.DataLen = 1;
    send_obj.Data[0] = 0x08;   // 读取命令

    // 发送读取请求
    VCI_Transmit(VCI_USBCAN2, 0, CAN_CHANNEL, &send_obj, 1);

    usleep(30000); // 等待返回

    ULONG len = VCI_Receive(VCI_USBCAN2, 0, CAN_CHANNEL, rec_obj, 50, 100);

    if (len <= 0) {
        printf("未收到数据\n");
        return 0;
    }

    for (ULONG i = 0; i < len; i++) {
        if (rec_obj[i].ID == motor_id && rec_obj[i].DataLen >= 5) {
            if (rec_obj[i].Data[0] == 0x08) {

                int32_t cnt;
                memcpy(&cnt, &rec_obj[i].Data[1], 4);

                float angle = cnt_to_angle(cnt);

                printf("当前角度: %.2f° (cnt=%d)\n", angle, cnt);
                return angle;
            }
        }
    }

    printf("解析失败\n");
    return 0;
}