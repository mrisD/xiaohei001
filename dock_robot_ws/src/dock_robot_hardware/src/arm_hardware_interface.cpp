#include "dock_robot_hardware/arm_hardware_interface.hpp"

namespace arm_hardware
{
    hardware_interface::CallbackReturn ArmHardwareInterface::
        on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        _can_ptr=std::make_unique<CAN_DRIVER>();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::
        on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        //Todo 设置初始值
        for(int i=0;i<10;i++){//初始化变量
            _jnt_position_command[i] = 0;
            _jnt_velocity_command[i] = 0;
            _jnt_torque_command[i] = 0;
            _jnt_position_state[i] = 0;
            _jnt_velocity_state[i] = 0;
            _jnt_torque_state[i] = 0;
        }
        _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
        m_isCANOpened=_can_ptr->can_open();
        if (!m_isCANOpened) {
            RCLCPP_INFO(rclcpp::get_logger("my_hardware"), "CAN 设备未连接，运行模拟模式+++++++++++++++++++++++++");
            return hardware_interface::CallbackReturn::FAILURE;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("my_hardware"), "CAN 初始化成功+++++++++++++++");

            const unsigned int motor_ids[10] = {
                LEFT_MOTOR1, LEFT_MOTOR2, LEFT_MOTOR3, LEFT_MOTOR4, LEFT_MOTOR5,
                RIGHT_MOTOR1, RIGHT_MOTOR2, RIGHT_MOTOR3, RIGHT_MOTOR4, RIGHT_MOTOR5
            };
            bool synced[10] = {};
            int remaining = 10;
            _can_ptr->update_states();
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);

            while (remaining > 0 && std::chrono::steady_clock::now() < deadline) {
                for (int i = 0; i < 10; ++i) {
                    if (synced[i]) {
                        continue;
                    }
                    const MotorState state = _can_ptr->motor_get_state(motor_ids[i]);
                    if (!state.updated) {
                        continue;
                    }
                    _jnt_position_state[i] = state.position_deg * M_PI / 180.0;
                    _jnt_position_command[i] = _jnt_position_state[i];
                    synced[i] = true;
                    --remaining;
                }
                if (remaining > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            }

            if (remaining > 0) {
                RCLCPP_ERROR(rclcpp::get_logger("my_hardware"), "电机初始位置读取失败，硬件插件激活失败");
                _can_ptr->can_close();
                m_isCANOpened = false;
                return hardware_interface::CallbackReturn::FAILURE;
            }
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::
        on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
        //Todo 关闭硬件，断开连接等
        _can_ptr->can_close();
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::
        on_init(const hardware_interface::HardwareInfo &info)
    {
        if(hardware_interface::SystemInterface::on_init(info) != 
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        info_=info;//info_是父类中定义的变量
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArmHardwareInterface::
        read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        (void)time;
        (void)period;
        if(m_isCANOpened)
        {
            // 非阻塞读取最新值
            _jnt_position_state[0] = _can_ptr->motor_get_angle(LEFT_MOTOR1) * M_PI / 180.0;
            _jnt_position_state[1] = _can_ptr->motor_get_angle(LEFT_MOTOR2) * M_PI / 180.0;
            _jnt_position_state[2] = _can_ptr->motor_get_angle(LEFT_MOTOR3) * M_PI / 180.0;
            _jnt_position_state[3] = _can_ptr->motor_get_angle(LEFT_MOTOR4) * M_PI / 180.0;
            _jnt_position_state[4] = _can_ptr->motor_get_angle(LEFT_MOTOR5) * M_PI / 180.0;

            _jnt_position_state[5] = _can_ptr->motor_get_angle(RIGHT_MOTOR1) * M_PI / 180.0;
            _jnt_position_state[6] = _can_ptr->motor_get_angle(RIGHT_MOTOR2) * M_PI / 180.0;
            _jnt_position_state[7] = _can_ptr->motor_get_angle(RIGHT_MOTOR3) * M_PI / 180.0;
            _jnt_position_state[8] = _can_ptr->motor_get_angle(RIGHT_MOTOR4) * M_PI / 180.0;
            _jnt_position_state[9] = _can_ptr->motor_get_angle(RIGHT_MOTOR5) * M_PI / 180.0;
        }  
        // RCLCPP_INFO(get_logger(), "STATE arm_base_forearm_joint_pos: %lf, forearm_hand_joint_pos: %lf",
        //             get_state("arm_base_forearm_joint/position"), get_state("forearm_hand_joint/position"));

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArmHardwareInterface::
        write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        if(!m_isCANOpened)
        {
            // return hardware_interface::return_type::ERROR;
        }
        else
        {
            if(_control_mode == 0)//位置控制模式
            {
                if(m_isCANOpened)
                {
                    // 批量发送所有电机命令
                    std::map<unsigned int, float> commands;
                    commands[LEFT_MOTOR1] = _jnt_position_command[0] * 180.0 / M_PI;
                    commands[LEFT_MOTOR2] = _jnt_position_command[1] * 180.0 / M_PI;
                    commands[LEFT_MOTOR3] = _jnt_position_command[2] * 180.0 / M_PI;
                    commands[LEFT_MOTOR4] = _jnt_position_command[3] * 180.0 / M_PI;
                    commands[LEFT_MOTOR5] = _jnt_position_command[4] * 180.0 / M_PI;

                    commands[RIGHT_MOTOR1] = _jnt_position_command[5] * 180.0 / M_PI;
                    commands[RIGHT_MOTOR2] = _jnt_position_command[6] * 180.0 / M_PI;
                    commands[RIGHT_MOTOR3] = _jnt_position_command[7] * 180.0 / M_PI;
                    commands[RIGHT_MOTOR4] = _jnt_position_command[8] * 180.0 / M_PI;
                    commands[RIGHT_MOTOR5] = _jnt_position_command[9] * 180.0 / M_PI;
                    
                    _can_ptr->motor_set_angle_batch(commands);
                }
                else
                {
                    // arm_base_forearm_joint_cmd_pos=get_command("arm_base_forearm_joint/position");
                    // forearm_hand_joint_cmd_pos=get_command("forearm_hand_joint/position");
                }
            }
        }


        
        

        // RCLCPP_INFO(rclcpp::get_logger("my_hardware"), "COMMAND arm_base_forearm_joint: %lf, forearm_hand_joint: %lf",
        //             _jnt_position_command[0]/M_PI*180, _jnt_position_command[1]/M_PI*180);
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> ArmHardwareInterface::
        export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        //导出关节相关的状态接口(位置，速度，扭矩)
        for (size_t i = 0; i < info_.joints.size(); ++i){
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

            // state_interfaces.emplace_back(hardware_interface::StateInterface(
            //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state[i]));

            // state_interfaces.emplace_back(hardware_interface::StateInterface(
            //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
        }

        //导出
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::
        export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));
            //     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
            //         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
        }

        return command_interfaces;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface,hardware_interface::SystemInterface)
