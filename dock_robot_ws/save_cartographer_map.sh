#!/bin/bash
# 保存地图脚本 (ROS2 Humble 版本)

echo "Finishing trajectory..."
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

sleep 1

# 定义保存路径，记得修改
MAP_PATH="/home/zhul/map/my_map"

echo "Saving .pbstream file..."
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${MAP_PATH}.pbstream'}"

echo "Converting to ROS map..."
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -map_filestem=${MAP_PATH} \
    -pbstream_filename=${MAP_PATH}.pbstream \
    -resolution=0.05

echo "Map saved to ${MAP_PATH}.pgm and .yaml"
