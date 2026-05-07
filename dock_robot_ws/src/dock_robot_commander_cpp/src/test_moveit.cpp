#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
using namespace std::chrono_literals;
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    

    auto node=std::make_shared<rclcpp::Node>("test_node");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spinner=std::thread([&executor](){executor.spin();});

    // rclcpp::sleep_for(std::chrono::seconds(5));

    auto right_arm=moveit::planning_interface::MoveGroupInterface(node,"right_arm");
    right_arm.startStateMonitor(); // 显式启动状态监控器
    right_arm.setMaxVelocityScalingFactor(1.0);
    right_arm.setMaxAccelerationScalingFactor(1.0);

    //Named Goal
    {
        right_arm.setStartStateToCurrentState();
        right_arm.setNamedTarget("right_arm_pose1");
        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        bool success1=(right_arm.plan(plan1)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success1)
        {
            right_arm.execute(plan1);
        }
    }
    rclcpp::sleep_for(5000ms);
    {
        right_arm.setStartStateToCurrentState();
        right_arm.setNamedTarget("right_arm_home");
        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        bool success1=(right_arm.plan(plan1)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success1)
        {
            right_arm.execute(plan1);
        }
    }   

    //-------------------------------------------------------------------------------
    // //Joint Goal
    // std::vector<double> joints={1.5, 0.5, 0.0, 1.5, 0.0, -1.7};
    // right_arm.setStartStateToCurrentState();
    // right_arm.setJointValueTarget(joints);
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1=(right_arm.plan(plan1)==moveit::core::MoveItErrorCode::SUCCESS);
    // if(success1)
    // {
    //     right_arm.execute(plan1);
    // }    

    //-------------------------------------------------------------------------------
    // //Pose Goal
    // tf2::Quaternion q;
    // q.setRPY(3.14, 0.0, 0.0);
    // q=q.normalize();

    // geometry_msgs::msg::PoseStamped target_pose;
    // target_pose.header.frame_id = "base_link";
    // target_pose.pose.position.x = 0.0;
    // target_pose.pose.position.y = -0.7;
    // target_pose.pose.position.z = 0.4;
    // target_pose.pose.orientation.x = q.getX();
    // target_pose.pose.orientation.y = q.getY();
    // target_pose.pose.orientation.z = q.getZ();
    // target_pose.pose.orientation.w = q.getW();
    
    // right_arm.setStartStateToCurrentState();
    // right_arm.setPoseTarget(target_pose);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1=(right_arm.plan(plan1)==moveit::core::MoveItErrorCode::SUCCESS);
    // if(success1)
    // {
    //     right_arm.execute(plan1);
    // }   

    // //Cartesian Path
    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // geometry_msgs::msg::Pose pose1 = right_arm.getCurrentPose().pose;
    // pose1.position.z += -0.2;
    // waypoints.push_back(pose1);
    // geometry_msgs::msg::Pose pose2 = pose1;
    // pose2.position.y += 0.2;
    // waypoints.push_back(pose2); 
    // geometry_msgs::msg::Pose pose3 = pose2;
    // pose3.position.y += -0.2;
    // pose3.position.z += 0.2;
    // waypoints.push_back(pose3);

    // moveit_msgs::msg::RobotTrajectory trajectory;

    // double fraction = right_arm.computeCartesianPath(waypoints, 0.01,0.0, trajectory);
    // if(fraction == 1)
    // {
    //     right_arm.execute(trajectory);
    // }
    // printf("fraction is %f +++++++++++++++++++=",fraction);
    rclcpp::shutdown();
    spinner.join();
    return 0;
}