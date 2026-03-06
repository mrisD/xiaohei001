#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node=std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner=std::thread([&](){
        executor.spin();
    });

    auto left_arm =moveit::planning_interface::MoveGroupInterface(node,"left_arm");
    left_arm.setMaxVelocityScalingFactor(1.0);
    left_arm.setMaxAccelerationScalingFactor(1.0);

    // //Named Goal
    // left_arm.setStartStateToCurrentState();
    // left_arm.setNamedTarget("left_arm_pose1");
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (left_arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success1)
    // {
    //     left_arm.execute(plan1);
    // }

    // left_arm.setStartStateToCurrentState();
    // left_arm.setNamedTarget("left_arm_home");
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (left_arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success2)
    // {
    //     left_arm.execute(plan2);
    // }
    // //-------------------------------------------------------------
    // //Joint Goal
    // std::vector<double> joints={1.5,0.5,1.5,0.5,1.5};
    // left_arm.setStartStateToCurrentState();
    // left_arm.setJointValueTarget(joints);
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (left_arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success1)
    // {
    //     left_arm.execute(plan1);
    // }

    //------------------------------------------------------------
    //Pose Goal    
    tf2::Quaternion q;
    q.setRPY(0.49729,0.44709,0.69818);
    q=q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id="arm_base_link";
    target_pose.pose.position.x=0.38247;
    target_pose.pose.position.y=0.17095;
    target_pose.pose.position.z=-0.30462;
    // target_pose.pose.orientation.w=q.getW();
    // target_pose.pose.orientation.x=q.getX();
    // target_pose.pose.orientation.y=q.getY();
    // target_pose.pose.orientation.z=q.getZ();
    
    target_pose.pose.orientation.x=0.497291;
    target_pose.pose.orientation.y=0.447101;
    target_pose.pose.orientation.z=0.255864;
    target_pose.pose.orientation.w=0.698094;
    left_arm.setStartStateToCurrentState();
    left_arm.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (left_arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success1)
    {
        left_arm.execute(plan1);
    }
    else
    {
        std::cout<<"规划失败"<<std::endl;
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}