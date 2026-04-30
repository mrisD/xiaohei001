#ifndef __CAN_DRIVER_HPP__
#define __CAN_DRIVER_HPP__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cstdint>
#include "controlcan.h"

/*配置*/
// #define MOTOR_CAN_ID      0x05
#define REDUCTION_RATIO   101
#define ENCODER_CNT       65536

#define VCI_USBCAN2       4
#define STATUS_OK         1
#define CAN_CHANNEL       1


#define RIGHT_MOTOR1 0x06
#define RIGHT_MOTOR2 0x07
#define RIGHT_MOTOR3 0x08
#define RIGHT_MOTOR4 0x09
#define RIGHT_MOTOR5 0x05
class CAN_DRIVER
{
public:
    CAN_DRIVER();
    ~CAN_DRIVER();

    VCI_INIT_CONFIG config;
    VCI_CAN_OBJ send_obj;
    VCI_CAN_OBJ rec_obj[50];
    /*转换*/
    int32_t angle_to_cnt(float angle);

    float cnt_to_angle(int32_t cnt);

    bool can_open();

    bool can_close();

    /*设置角度*/
    bool motor_set_angle(unsigned int motor_id,float angle);

    /*读取当前位置*/
    float motor_read_angle(unsigned int motor_id);

private:
    bool is_canOpened=false;

};


#endif