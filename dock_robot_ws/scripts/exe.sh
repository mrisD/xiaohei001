#!/bin/bash
rm -rf build/ install/ log/
colcon build
export ROS_DOMAIN_ID=51
source install/setup.bash
sleep 5
ros2 launch dock_robot_moveit_config gazebo.launch.py
