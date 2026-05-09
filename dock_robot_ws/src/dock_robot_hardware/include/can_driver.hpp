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
#include "controlcan.h"

#define REDUCTION_RATIO   101
#define ENCODER_CNT       65536
#define VCI_USBCAN2       4
#define STATUS_OK         1
#define CAN_CHANNEL       1

// 电机ID定义

#define LEFT_MOTOR1 0x01
#define LEFT_MOTOR2 0x02
#define LEFT_MOTOR3 0x03
#define LEFT_MOTOR4 0x04
#define LEFT_MOTOR5 0x010

#define RIGHT_MOTOR1 0x06
#define RIGHT_MOTOR2 0x07
#define RIGHT_MOTOR3 0x08
#define RIGHT_MOTOR4 0x09
#define RIGHT_MOTOR5 0x05

// 电机状态结构体
struct MotorState {
    float position_deg = 0.0;  // 角度（度）
    float velocity = 0.0;       // 速度
    uint32_t timestamp = 0;     // 时间戳
    bool updated = false;       // 是否更新
};

class CAN_DRIVER
{
public:
    CAN_DRIVER();
    ~CAN_DRIVER();

    bool can_open();
    bool can_close();
    
    // 异步发送命令（不等待回复）
    bool motor_set_angle(unsigned int motor_id, float angle);
    bool motor_set_angle_batch(const std::map<unsigned int, float>& commands);
    
    // 获取最新缓存的状态（非阻塞）
    float motor_get_angle(unsigned int motor_id);
    MotorState motor_get_state(unsigned int motor_id);
    
    // 更新所有电机状态（非阻塞）
    void update_states();

private:
    void receive_thread_func();     // 接收线程
    void send_pending_commands();   // 发送待处理命令
    
    VCI_INIT_CONFIG config;
    VCI_CAN_OBJ send_obj;
    VCI_CAN_OBJ recv_buffers[100];  // 接收缓冲区
    
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_canOpened_{false};
    
    std::thread receive_thread_;
    std::mutex state_mutex_;
    std::map<unsigned int, MotorState> motor_states_;
    
    // 命令队列（可选，用于更平滑的控制）
    std::mutex cmd_mutex_;
    std::map<unsigned int, float> pending_commands_;

    // 添加定时查询相关
    std::chrono::steady_clock::time_point last_query_time_;
    std::chrono::milliseconds query_interval_{20};  // 默认20ms查询一次
    
    // 辅助函数
    int32_t angle_to_cnt(float angle);
    float cnt_to_angle(int32_t cnt);
    void parse_can_message(const VCI_CAN_OBJ& obj);
};

#endif