#ifndef __CAN_DRIVER_HPP__
#define __CAN_DRIVER_HPP__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cstdint>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

#include "controlcan.h"

// ==========================
// 电机参数
// ==========================
#define REDUCTION_RATIO   101
#define ENCODER_CNT       65536

// ==========================
// CAN 设备参数
// ==========================
#define VCI_USBCAN2       4
#define STATUS_OK         1

// ==========================
// CAN 通道定义
// ==========================
// CAN1 -> 左臂
// CAN2 -> 右臂
#define CAN_CHANNEL_LEFT        0
#define CAN_CHANNEL_RIGHT       1

// ==========================
// 左臂电机 ID
// ==========================
#define LEFT_MOTOR1 0x01
#define LEFT_MOTOR2 0x02
#define LEFT_MOTOR3 0x03
#define LEFT_MOTOR4 0x04
#define LEFT_MOTOR5 0x10

// ==========================
// 右臂电机 ID
// ==========================
#define RIGHT_MOTOR1 0x06
#define RIGHT_MOTOR2 0x07
#define RIGHT_MOTOR3 0x08
#define RIGHT_MOTOR4 0x09
#define RIGHT_MOTOR5 0x05

// ==========================
// 电机状态结构体
// ==========================
struct MotorState {
    float position_deg = 0.0f;  // 角度，单位：度
    float velocity = 0.0f;      // 速度，当前程序暂未解析
    uint32_t timestamp = 0;     // 时间戳
    bool updated = false;       // 是否收到过更新
};

class CAN_DRIVER
{
public:
    CAN_DRIVER();
    ~CAN_DRIVER();

    bool can_open();
    bool can_close();

    // 异步发送命令，不等待回复
    bool motor_set_angle(unsigned int motor_id, float angle);
    bool motor_set_angle_batch(const std::map<unsigned int, float>& commands);

    // 获取最新缓存状态，非阻塞
    float motor_get_angle(unsigned int motor_id);
    MotorState motor_get_state(unsigned int motor_id);

    // 主动查询所有电机状态，非阻塞
    void update_states();

private:
    // 接收线程
    void receive_thread_func();

    // 发送待处理命令
    void send_pending_commands();

    // 根据电机 ID 判断走哪个 CAN 通道
    int get_motor_channel(unsigned int motor_id) const;

    // 电机角度和编码器计数转换
    int32_t angle_to_cnt(float angle);
    float cnt_to_angle(int32_t cnt);

    // 解析 CAN 返回数据
    void parse_can_message(const VCI_CAN_OBJ& obj);

private:
    VCI_INIT_CONFIG config;
    VCI_CAN_OBJ recv_buffers_[100];

    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_canOpened_{false};

    std::thread receive_thread_;

    // 电机状态缓存
    std::mutex state_mutex_;
    std::map<unsigned int, MotorState> motor_states_;

    // 命令缓存
    std::mutex cmd_mutex_;
    std::map<unsigned int, float> pending_commands_;

    // 定时查询周期
    std::chrono::milliseconds query_interval_{20};
};

#endif