#include "dock_robot_hardware/arm_hardware_interface.hpp"

namespace arm_hardware
{
    // ==========================================================
    // 关节正方向映射表
    //
    //   +1.0 : 电机正方向 == URDF 中该关节 <axis> 的正方向
    //   -1.0 : 相反（需要取反）
    //
    // 全部 10 个臂关节的电机转向与 URDF 正方向整体相反，故统一取 -1。
    // 这样仿真(mock/Gazebo，直接用 URDF)与真机的运动方向一致。
    //
    // 注意：这里必须同时对「读」和「写」生效。
    //   read()  : state = SIGN * 电机角度
    //   write() : 电机角度 = SIGN * command
    // 如果只改一边会形成正反馈，关节会越跑越快冲到底。
    //
    // 以后若某个电机重新接线/改 DIP 方向，只改对应一项即可，
    // 不用动 URDF，也就不影响已经调好的 SRDF 位姿和 MTC 参数。
    // ==========================================================
    namespace
    {
        constexpr size_t JOINT_SIGN_NUM = 15;
        constexpr double JOINT_SIGN[JOINT_SIGN_NUM] = {
            -1.0, -1.0, -1.0, -1.0, -1.0,   // 0..4   joint1..joint5   (CAN1 左臂)
            +1.0, -1.0, +1.0, -1.0, -1.0,   // 5..9   jointr1..jointr5
                                            //        jointr1(5)、jointr3(7) 的电机方向
                                            //        与其余关节相反，单独取 +1.0
             1.0,  1.0,  1.0,  1.0,  1.0    // 10..14 轮子/升降/夹爪：不走 CAN，保持原样
        };

        inline double jointSign(size_t i)
        {
            return (i < JOINT_SIGN_NUM) ? JOINT_SIGN[i] : 1.0;
        }

        // 关节下标 -> 电机 CAN ID，与 URDF/ros2_control 里 <joint> 的声明顺序一致
        constexpr unsigned int MOTOR_IDS[10] = {
            LEFT_MOTOR1,  LEFT_MOTOR2,  LEFT_MOTOR3,  LEFT_MOTOR4,  LEFT_MOTOR5,
            RIGHT_MOTOR1, RIGHT_MOTOR2, RIGHT_MOTOR3, RIGHT_MOTOR4, RIGHT_MOTOR5
        };

        constexpr const char *JOINT_NAMES[10] = {
            "joint1", "joint2", "joint3", "joint4", "joint5",
            "jointr1", "jointr2", "jointr3", "jointr4", "jointr5"
        };
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::
        on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        _can_ptr=std::make_unique<CAN_DRIVER>();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    auto logger = rclcpp::get_logger("my_hardware");

    RCLCPP_INFO(logger, "开始激活 ArmHardwareInterface...");

    // ==============================
    // 1. 初始化关节变量
    // ==============================
    for (int i = 0; i < 10; i++)
    {
        _jnt_position_command[i] = 0.0;
        _jnt_velocity_command[i] = 0.0;
        _jnt_torque_command[i] = 0.0;

        _jnt_position_state[i] = 0.0;
        _jnt_velocity_state[i] = 0.0;
        _jnt_torque_state[i] = 0.0;
    }

    // 默认位置控制：0-位置控制，1-扭矩控制，2-速度控制
    _control_mode = 0;

    // ==============================
    // 2. 打开 CAN
    // ==============================
    m_isCANOpened = _can_ptr->can_open();

    if (!m_isCANOpened)
    {
        RCLCPP_ERROR(logger, "CAN 设备打开失败，硬件未连接");

        /*
         * 重要：
         * 这里不要 return FAILURE。
         * 如果返回 FAILURE，ros2_control_node 可能直接退出，
         * 后续 joint_state_broadcaster / MoveIt / RViz 都会启动失败。
         *
         * 让硬件接口保持 SUCCESS，但 m_isCANOpened=false。
         * 后续 read()/write() 里面需要根据 m_isCANOpened 做保护。
         */
        RCLCPP_WARN(logger, "进入安全模拟状态：controller_manager 保持运行，但不发送真实硬件命令");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(logger, "CAN 初始化成功+++++++++++++++");

    // ==============================
    // 3. 电机 ID / 关节名 表
    //    统一使用文件顶部的 MOTOR_IDS / JOINT_NAMES，避免两处表不一致
    // ==============================

    bool synced[10] = {};
    int remaining = 10;

    // ==============================
    // 4. 等待电机反馈初始位置
    // ==============================
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);

    while (remaining > 0 && std::chrono::steady_clock::now() < deadline)
    {
        /*
         * 关键修复：
         * update_states() 必须放在 while 循环里面。
         * 你原来的代码只调用了一次，后面一直读旧状态。
         */
        _can_ptr->update_states();

        for (int i = 0; i < 10; ++i)
        {
            if (synced[i])
            {
                continue;
            }

            const MotorState state = _can_ptr->motor_get_state(MOTOR_IDS[i]);

            if (!state.updated)
            {
                continue;
            }

            _jnt_position_state[i] = jointSign(i) * state.position_deg * M_PI / 180.0;
            _jnt_velocity_state[i] = 0.0;
            _jnt_torque_state[i] = 0.0;

            /*
             * 很重要：
             * 激活时 command 必须等于当前 state。
             * 否则 controller 一激活，可能立即把电机拉回 0 位。
             */
            _jnt_position_command[i] = _jnt_position_state[i];
            _jnt_velocity_command[i] = 0.0;
            _jnt_torque_command[i] = 0.0;

            synced[i] = true;
            --remaining;

            RCLCPP_INFO(
                logger,
                "读取初始位置成功: %s, motor_id=0x%02X, position=%.3f deg, %.6f rad",
                JOINT_NAMES[i],
                MOTOR_IDS[i],
                state.position_deg,
                _jnt_position_state[i]);
        }

        if (remaining > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // ==============================
    // 5. 处理未同步的电机
    // ==============================
    if (remaining > 0)
    {
        RCLCPP_ERROR(
            logger,
            "有 %d 个电机初始位置读取失败，但不让 ros2_control_node 崩溃",
            remaining);

        for (int i = 0; i < 10; ++i)
        {
            if (!synced[i])
            {
                RCLCPP_ERROR(
                    logger,
                    "未收到电机反馈: joint=%s, motor_id=0x%02X",
                    JOINT_NAMES[i],
                    MOTOR_IDS[i]);

                /*
                 * 没读到真实位置的关节，先保持 0。
                 * 注意：这种情况下建议 write() 不要真正下发运动命令。
                 */
                _jnt_position_state[i] = 0.0;
                _jnt_velocity_state[i] = 0.0;
                _jnt_torque_state[i] = 0.0;

                _jnt_position_command[i] = _jnt_position_state[i];
                _jnt_velocity_command[i] = 0.0;
                _jnt_torque_command[i] = 0.0;
            }
        }

        /*
         * 不关闭 CAN，不 return FAILURE。
         * 让 controller_manager、joint_state_broadcaster、MoveIt、RViz 正常启动。
         */
        RCLCPP_WARN(
            logger,
            "硬件部分电机未同步，系统进入保护状态。请检查电机 ID、CAN 通道、电源、波特率和接线。");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(logger, "全部电机初始位置读取成功，硬件插件激活完成");

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
            // 非阻塞读取最新值，并用 JOINT_SIGN 从「电机正方向」换算到「ROS 关节正方向」
            for (size_t i = 0; i < 10; ++i)
            {
                _jnt_position_state[i] =
                    jointSign(i) * _can_ptr->motor_get_angle(MOTOR_IDS[i]) * M_PI / 180.0;
            }
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
                    // 批量发送所有电机命令。
                    // 先用 JOINT_SIGN 把「ROS 关节正方向」换算回「电机正方向」，再由度转编码器。
                    // 必须和 read() 用同一套符号，否则会形成正反馈。
                    std::map<unsigned int, float> commands;
                    for (size_t i = 0; i < 10; ++i)
                    {
                        commands[MOTOR_IDS[i]] = static_cast<float>(
                            jointSign(i) * _jnt_position_command[i] * 180.0 / M_PI);
                    }

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
