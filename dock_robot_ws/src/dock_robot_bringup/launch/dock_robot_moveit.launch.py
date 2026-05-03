#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():

    # 包路径
    dock_robot_description = FindPackageShare('dock_robot_moveit_config').find('dock_robot_moveit_config')
    dock_robot_bringup = FindPackageShare('dock_robot_bringup').find('dock_robot_bringup')
    dock_robot_description2 = FindPackageShare('dock_robot_description').find('dock_robot_description')

    # 文件路径
    urdf_path = os.path.join(dock_robot_description, 'config', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(dock_robot_description2, 'rviz', 'moveit_config.rviz')
    controllers_config_path = os.path.join(dock_robot_bringup, 'config', 'dock_robot_controllers.yaml')
    # URDF
    robot_description = Command(['xacro ', urdf_path])

    #robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    #ros2_control_node（关键：必须带 robot_description）
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_config_path
        ],
        output='screen'
    )

    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="dock_robot_moveit_config")
        .robot_description(file_path='config/my_robot.urdf.xacro')
        .robot_description_semantic(file_path='config/my_robot.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,           # 在这里设置
            publish_robot_description_semantic=True   # 在这里设置
        )
        .to_moveit_configs()
    )

    # 获取配置字典
    moveit_config_dict = moveit_config.to_dict()



    moveit_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict,{'use_sim_time': False}],
    )

    #延迟启动 controllers（Humble必须）
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
            moveit_group_node,
        ]
    )



    

    # RViz 节点也需要相同的 kinematics 配置！
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[moveit_config_dict,{'use_sim_time': False}],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        rviz2,
    ])