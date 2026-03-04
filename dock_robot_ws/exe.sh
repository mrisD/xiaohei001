#!/bin/bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
#ros2 launch robot_arm_config gazebo.launch.py
