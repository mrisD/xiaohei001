#!/bin/bash
cd
cd dock_robot_ws/
source install/setup.bash
ros2 launch dock_robot_moveit_config demo.launch.py
