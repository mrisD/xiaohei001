#!/bin/bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mobile_controller/cmd_vel -p stamped:=true
