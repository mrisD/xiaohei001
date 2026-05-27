#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 包路径
    dock_robot_moveit_config = FindPackageShare(
        'dock_robot_moveit_config'
    ).find('dock_robot_moveit_config')

    dock_robot_bringup = FindPackageShare(
        'dock_robot_bringup'
    ).find('dock_robot_bringup')

    dock_robot_description = FindPackageShare(
        'dock_robot_description'
    ).find('dock_robot_description')

    # 文件路径
    urdf_path = os.path.join(
        dock_robot_moveit_config,
        'config',
        'my_robot.urdf.xacro'
    )

    controllers_config_path = os.path.join(
        dock_robot_bringup,
        'config',
        'dock_robot_controllers.yaml'
    )

    # 注意：这里先用 urdf_config.rviz，不要用 moveit_config.rviz
    rviz_config_path = os.path.join(
        dock_robot_description,
        'rviz',
        'urdf_config.rviz'
    )

    # URDF
    robot_description = Command([
        'xacro ',
        urdf_path
    ])

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    # ros2_control_node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            },
            controllers_config_path
        ]
    )

    # controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'body_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    mobile_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
            body_controller_spawner,
            mobile_controller_spawner,
        ]
    )

    # RViz 延迟启动，避免 joint_states 还没出来就加载模型
    delayed_rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d',
                    rviz_config_path
                ],
                parameters=[
                    {
                        'robot_description': robot_description
                    }
                ]
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        delayed_rviz,
    ])