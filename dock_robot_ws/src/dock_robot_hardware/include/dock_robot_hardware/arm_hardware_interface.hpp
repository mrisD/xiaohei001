#ifndef __ARM_HARDWARE_INTERFACE_HPP__
#define __ARM_HARDWARE_INTERFACE_HPP__

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "can_driver.hpp"
#include "rclcpp/rclcpp.hpp"
namespace arm_hardware
{
class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    // System interface override
    hardware_interface::CallbackReturn 
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type 
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type 
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    std::vector<hardware_interface::StateInterface> 
        export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> 
        export_command_interfaces() override;

private:
    std::unique_ptr<CAN_DRIVER> _can_ptr;
    bool m_isCANOpened=false;

    double _jnt_position_command[15];
    double _jnt_velocity_command[15];
    double _jnt_torque_command[15];
    double _jnt_position_state[15];
    double _jnt_velocity_state[15];
    double _jnt_torque_state[15];
    int _control_mode;
};    
}






#endif