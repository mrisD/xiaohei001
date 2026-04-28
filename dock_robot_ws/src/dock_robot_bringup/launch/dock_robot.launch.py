#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 包路径
    dock_robot_description = FindPackageShare('dock_robot_moveit_config').find('dock_robot_moveit_config')
    dock_robot_bringup = FindPackageShare('dock_robot_bringup').find('dock_robot_bringup')
    dock_robot_description2 = FindPackageShare('dock_robot_description').find('dock_robot_description')

    # 文件路径
    urdf_path = os.path.join(dock_robot_description, 'config', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(dock_robot_description2, 'rviz', 'urdf_config.rviz')
    controllers_config_path = os.path.join(dock_robot_bringup, 'config', 'dock_robot_controllers.yaml')

    # URDF
    robot_description = Command(['xacro ', urdf_path])

    # 1️⃣ robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 2️⃣ ros2_control_node（⚠️关键：必须带 robot_description）
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_config_path
        ],
        output='screen'
    )

    # ⏱ 延迟启动 controllers（Humble必须）
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            ),

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'left_arm_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            ),

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'right_arm_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            ),

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'body_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            ),

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'mobile_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            ),
        ]
    )

    # 4️⃣ RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        rviz2,
    ])