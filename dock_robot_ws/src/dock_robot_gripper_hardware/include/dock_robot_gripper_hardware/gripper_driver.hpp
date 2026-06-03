#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <termios.h>

namespace dock_robot_gripper_hardware
{

class GripperDriver
{
public:
  struct MotorStatus
  {
    uint8_t address{0};
    bool reached{false};
    float speed{0.0F};
    uint16_t speed_raw{0};
    int position{0};
  };

  GripperDriver() = default;
  GripperDriver(const GripperDriver &) = delete;
  GripperDriver & operator=(const GripperDriver &) = delete;
  ~GripperDriver();

  bool openPort(const std::string & port, int baudrate, std::string * error = nullptr);
  void closePort();
  bool isOpen() const;

  bool open(uint16_t speed = 80);
  bool close(uint16_t speed = 80);
  bool move(uint8_t direction, uint16_t speed, uint16_t position_count);

  bool getStatus(MotorStatus & status, int timeout_ms = 5, int position_low_index = 5);

private:
  int fd_{-1};

  static speed_t baudrateToTermios(int baudrate);
  static uint8_t xorCheck(const std::vector<uint8_t> & data);
  static uint16_t twoBytesToUint16(uint8_t high, uint8_t low);

  bool sendFrame(uint8_t direction, uint16_t speed, uint8_t pos_high, uint8_t pos_low);
  bool writeAll(const uint8_t * data, size_t size);
  bool requestFeedback(std::vector<uint8_t> & response, int timeout_ms);
  bool parseFeedback(const std::vector<uint8_t> & data, MotorStatus & status, int position_low_index);
};

}  // namespace dock_robot_gripper_hardware
