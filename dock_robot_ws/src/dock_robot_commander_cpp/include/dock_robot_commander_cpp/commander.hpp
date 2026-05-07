#ifndef __COMMANDER_HPP__
#define __COMMANDER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <dock_robot_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = dock_robot_interfaces::msg::PoseCommand;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node);
    ~Commander();
    void goToNamedTarget(const std::string &name);
    void goToJointTarget(const std::vector<double> &joints);
    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false);
private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface);
    void jointCmdCallback(const FloatArray &msg);
    void poseCmdCallback(const PoseCmd &msg);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> right_arm_;

    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;


};


#endif 