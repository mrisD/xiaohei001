#include "dock_robot_commander_cpp/commander.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

Commander::Commander(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;
    right_arm_ = std::make_shared<MoveGroupInterface>(node_, "right_arm");
    right_arm_->setMaxVelocityScalingFactor(1.0);
    right_arm_->setMaxAccelerationScalingFactor(1.0);

    joint_cmd_sub_ = node_->create_subscription<FloatArray>("joint_command", 10,
                                                            std::bind(&Commander::jointCmdCallback, this, std::placeholders::_1));

    pose_cmd_sub_ = node_->create_subscription<PoseCmd>("pose_command", 10,
                                                        std::bind(&Commander::poseCmdCallback, this, std::placeholders::_1));
}

Commander::~Commander()
{
}

void Commander::goToNamedTarget(const std::string &name)
{
    right_arm_->setStartStateToCurrentState();
    right_arm_->setNamedTarget(name);
    planAndExecute(right_arm_);
}

void Commander::goToJointTarget(const std::vector<double> &joints)
{
    right_arm_->setStartStateToCurrentState();
    right_arm_->setJointValueTarget(joints);
    planAndExecute(right_arm_);
}

void Commander::goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q = q.normalize();
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();
    right_arm_->setStartStateToCurrentState();
    if (!cartesian_path)
    {
        right_arm_->setPoseTarget(target_pose);
        planAndExecute(right_arm_);
    }
    else
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = right_arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        if (fraction == 1)
        {
            right_arm_->execute(trajectory);
        }
    }
}

void Commander::planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
{
    MoveGroupInterface::Plan plan;
    bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
        interface->execute(plan);
    }
}

void Commander::jointCmdCallback(const FloatArray &msg)
{
    auto joints = msg.data;
    if(joints.size() ==5)
    {
        goToJointTarget(joints);
    }
}

void Commander::poseCmdCallback(const PoseCmd &msg)
{
    goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
}
