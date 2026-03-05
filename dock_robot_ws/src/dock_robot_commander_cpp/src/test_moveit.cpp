#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

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
    
    left_arm.setStartStateToCurrentState();
    left_arm.setNamedTarget("left_arm_pose1");
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (left_arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success1)
    {
        left_arm.execute(plan1);
    }

    left_arm.setStartStateToCurrentState();
    left_arm.setNamedTarget("left_arm_home");
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (left_arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success2)
    {
        left_arm.execute(plan2);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}