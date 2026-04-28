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
