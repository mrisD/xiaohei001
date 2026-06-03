#pragma once

#include "dock_robot_gripper_hardware/gripper_driver.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dock_robot_gripper_hardware
{

class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GripperHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string getParam(const std::string & name, const std::string & default_value) const;
  int getIntParam(const std::string & name, int default_value) const;
  double getDoubleParam(const std::string & name, double default_value) const;
  bool getBoolParam(const std::string & name, bool default_value) const;

  double clampJointPosition(double value) const;
  double binaryTarget(double command) const;
  uint16_t positionCountFromCommand(double target_position, double reference_position) const;
  double jointPositionFromRaw(int raw_position) const;
  uint8_t directionFromCommand(double target_position, double reference_position) const;
  bool sendTarget(double target_position, double reference_position);

  rclcpp::Logger logger_{rclcpp::get_logger("dock_robot_gripper_hardware")};
  std::unique_ptr<GripperDriver> gripper_;

  std::string joint_name_;
  std::string serial_port_{"/dev/ttyACM0"};
  int baudrate_{115200};
  int speed_{80};
  int feedback_timeout_ms_{5};
  int feedback_period_ms_{100};
  int feedback_position_low_index_{5};

  double min_joint_position_{0.0};
  double max_joint_position_{0.06};
  int max_position_count_{18720};
  int min_move_count_{10};
  double command_deadband_{0.0005};

  bool positive_is_close_{true};
  bool binary_control_{true};
  bool use_delta_position_count_{true};
  bool use_feedback_{false};
  bool simulate_when_not_connected_{true};

  bool connected_{false};
  bool active_{false};
  double hw_position_state_{0.0};
  double hw_position_command_{0.0};
  double last_sent_target_{0.0};

  std::chrono::steady_clock::time_point last_feedback_time_{};
  std::chrono::steady_clock::time_point last_feedback_warn_time_{};
};

}  // namespace dock_robot_gripper_hardware
