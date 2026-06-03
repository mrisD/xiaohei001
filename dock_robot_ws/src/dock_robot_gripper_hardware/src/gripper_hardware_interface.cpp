#include "dock_robot_gripper_hardware/gripper_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace dock_robot_gripper_hardware
{

hardware_interface::CallbackReturn GripperHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(logger_, "GripperHardwareInterface expects exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_name_ = info_.joints[0].name;

  if (info_.joints[0].command_interfaces.size() != 1 ||
      info_.joints[0].command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
    RCLCPP_ERROR(logger_, "Joint '%s' must expose exactly one position command interface", joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints[0].state_interfaces.size() != 1 ||
      info_.joints[0].state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
    RCLCPP_ERROR(logger_, "Joint '%s' must expose exactly one position state interface", joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = getParam("serial_port", serial_port_);
  baudrate_ = getIntParam("baudrate", baudrate_);
  speed_ = getIntParam("speed", speed_);
  feedback_timeout_ms_ = getIntParam("feedback_timeout_ms", feedback_timeout_ms_);
  feedback_period_ms_ = getIntParam("feedback_period_ms", feedback_period_ms_);
  feedback_position_low_index_ = getIntParam("feedback_position_low_index", feedback_position_low_index_);
  min_joint_position_ = getDoubleParam("min_joint_position", min_joint_position_);
  max_joint_position_ = getDoubleParam("max_joint_position", max_joint_position_);
  max_position_count_ = getIntParam("max_position_count", max_position_count_);
  min_move_count_ = getIntParam("min_move_count", min_move_count_);
  command_deadband_ = getDoubleParam("command_deadband", command_deadband_);
  positive_is_close_ = getBoolParam("positive_is_close", positive_is_close_);
  binary_control_ = getBoolParam("binary_control", binary_control_);
  use_delta_position_count_ = getBoolParam("use_delta_position_count", use_delta_position_count_);
  use_feedback_ = getBoolParam("use_feedback", use_feedback_);
  simulate_when_not_connected_ = getBoolParam("simulate_when_not_connected", simulate_when_not_connected_);

  if (max_joint_position_ <= min_joint_position_) {
    RCLCPP_ERROR(logger_, "max_joint_position must be greater than min_joint_position");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (max_position_count_ <= 0) {
    RCLCPP_ERROR(logger_, "max_position_count must be greater than 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ROS 2 Humble's hardware_interface::InterfaceInfo has no `parameters` member.
  // Keep the initial position as a hardware parameter for Humble compatibility.
  hw_position_state_ = clampJointPosition(getDoubleParam("initial_position", min_joint_position_));

  hw_position_command_ = hw_position_state_;
  last_sent_target_ = binary_control_ ? binaryTarget(hw_position_state_) : hw_position_state_;

  RCLCPP_INFO(
    logger_,
    "Configured gripper: joint=%s port=%s baudrate=%d speed=%d range=[%.4f, %.4f] count=%d binary=%s feedback=%s",
    joint_name_.c_str(), serial_port_.c_str(), baudrate_, speed_,
    min_joint_position_, max_joint_position_, max_position_count_,
    binary_control_ ? "true" : "false", use_feedback_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  gripper_ = std::make_unique<GripperDriver>();

  std::string error;
  connected_ = gripper_->openPort(serial_port_, baudrate_, &error);
  if (!connected_) {
    if (simulate_when_not_connected_) {
      RCLCPP_WARN(
        logger_,
        "Failed to open gripper serial port '%s': %s. Running in simulated/open-loop mode.",
        serial_port_.c_str(), error.c_str());
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_ERROR(
      logger_, "Failed to open gripper serial port '%s': %s", serial_port_.c_str(), error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Opened gripper serial port '%s'", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = true;
  hw_position_command_ = hw_position_state_;
  last_sent_target_ = binary_control_ ? binaryTarget(hw_position_state_) : hw_position_state_;
  last_feedback_time_ = std::chrono::steady_clock::now();
  last_feedback_warn_time_ = last_feedback_time_;

  RCLCPP_INFO(logger_, "Activated gripper hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  if (gripper_) {
    gripper_->closePort();
  }
  connected_ = false;

  RCLCPP_INFO(logger_, "Deactivated gripper hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_command_);
  return command_interfaces;
}

hardware_interface::return_type GripperHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  const auto now = std::chrono::steady_clock::now();

  if (connected_ && use_feedback_) {
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_time_).count();
    if (elapsed_ms >= feedback_period_ms_) {
      GripperDriver::MotorStatus status;
      if (gripper_->getStatus(status, feedback_timeout_ms_, feedback_position_low_index_)) {
        hw_position_state_ = jointPositionFromRaw(status.position);
        last_feedback_time_ = now;
      } else {
        const auto warn_elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_warn_time_).count();
        if (warn_elapsed_ms > 1000) {
          RCLCPP_WARN(logger_, "Failed to read gripper feedback, keeping last state %.4f", hw_position_state_);
          last_feedback_warn_time_ = now;
        }
      }
    }
  } else {
    // Open-loop state tracking. This is recommended first for MoveIt/JTC integration.
    hw_position_state_ = clampJointPosition(hw_position_command_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  const double command = clampJointPosition(hw_position_command_);
  const double target = binary_control_ ? binaryTarget(command) : command;

  if (std::fabs(target - last_sent_target_) < command_deadband_) {
    return hardware_interface::return_type::OK;
  }

  const double reference = last_sent_target_;
  if (!sendTarget(target, reference)) {
    return hardware_interface::return_type::ERROR;
  }

  last_sent_target_ = target;
  if (!use_feedback_) {
    hw_position_state_ = target;
  }

  return hardware_interface::return_type::OK;
}

std::string GripperHardwareInterface::getParam(
  const std::string & name, const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return default_value;
  }
  return it->second;
}

int GripperHardwareInterface::getIntParam(const std::string & name, const int default_value) const
{
  const auto text = getParam(name, "");
  if (text.empty()) {
    return default_value;
  }

  try {
    return std::stoi(text);
  } catch (const std::exception &) {
    RCLCPP_WARN(logger_, "Invalid int parameter '%s'='%s', using %d", name.c_str(), text.c_str(), default_value);
    return default_value;
  }
}

double GripperHardwareInterface::getDoubleParam(const std::string & name, const double default_value) const
{
  const auto text = getParam(name, "");
  if (text.empty()) {
    return default_value;
  }

  try {
    return std::stod(text);
  } catch (const std::exception &) {
    RCLCPP_WARN(logger_, "Invalid double parameter '%s'='%s', using %.6f", name.c_str(), text.c_str(), default_value);
    return default_value;
  }
}

bool GripperHardwareInterface::getBoolParam(const std::string & name, const bool default_value) const
{
  auto text = getParam(name, "");
  if (text.empty()) {
    return default_value;
  }

  std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (text == "true" || text == "1" || text == "yes" || text == "on") {
    return true;
  }
  if (text == "false" || text == "0" || text == "no" || text == "off") {
    return false;
  }

  RCLCPP_WARN(
    logger_, "Invalid bool parameter '%s'='%s', using %s",
    name.c_str(), text.c_str(), default_value ? "true" : "false");
  return default_value;
}

double GripperHardwareInterface::clampJointPosition(const double value) const
{
  return std::clamp(value, min_joint_position_, max_joint_position_);
}

double GripperHardwareInterface::binaryTarget(const double command) const
{
  const double middle = 0.5 * (min_joint_position_ + max_joint_position_);
  return command >= middle ? max_joint_position_ : min_joint_position_;
}

uint16_t GripperHardwareInterface::positionCountFromCommand(
  const double target_position, const double reference_position) const
{
  const double range = max_joint_position_ - min_joint_position_;
  double ratio = 0.0;

  if (binary_control_) {
    ratio = 1.0;
  } else if (use_delta_position_count_) {
    ratio = std::fabs(target_position - reference_position) / range;
  } else {
    ratio = (target_position - min_joint_position_) / range;
  }

  ratio = std::clamp(ratio, 0.0, 1.0);
  const auto count = static_cast<int>(std::lround(ratio * static_cast<double>(max_position_count_)));
  return static_cast<uint16_t>(std::clamp(count, 0, max_position_count_));
}

double GripperHardwareInterface::jointPositionFromRaw(const int raw_position) const
{
  const int clamped_raw = std::clamp(raw_position, 0, max_position_count_);
  const double ratio = static_cast<double>(clamped_raw) / static_cast<double>(max_position_count_);
  return clampJointPosition(min_joint_position_ + ratio * (max_joint_position_ - min_joint_position_));
}

uint8_t GripperHardwareInterface::directionFromCommand(
  const double target_position, const double reference_position) const
{
  const bool command_increasing = target_position >= reference_position;
  if (positive_is_close_) {
    return command_increasing ? 0x01 : 0x00;
  }
  return command_increasing ? 0x00 : 0x01;
}

bool GripperHardwareInterface::sendTarget(const double target_position, const double reference_position)
{
  const uint16_t count = positionCountFromCommand(target_position, reference_position);
  if (count < static_cast<uint16_t>(min_move_count_)) {
    return true;
  }

  const uint8_t direction = directionFromCommand(target_position, reference_position);

  if (!connected_) {
    if (simulate_when_not_connected_) {
      RCLCPP_WARN(
        logger_,
        "Gripper serial is not connected. Simulating command target=%.4f direction=0x%02X count=%u",
        target_position, direction, count);
      return true;
    }
    RCLCPP_ERROR(logger_, "Gripper serial is not connected");
    return false;
  }

  if (!gripper_->move(direction, static_cast<uint16_t>(speed_), count)) {
    RCLCPP_ERROR(
      logger_, "Failed to send gripper command target=%.4f direction=0x%02X count=%u",
      target_position, direction, count);
    return false;
  }

  RCLCPP_INFO(
    logger_, "Sent gripper command target=%.4f direction=0x%02X count=%u speed=%d",
    target_position, direction, count, speed_);
  return true;
}

}  // namespace dock_robot_gripper_hardware

PLUGINLIB_EXPORT_CLASS(
  dock_robot_gripper_hardware::GripperHardwareInterface,
  hardware_interface::SystemInterface)
