#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ==============================
    # 包路径查找
    # ==============================
    robot_bringup_share = FindPackageShare("dock_robot_bringup").find("dock_robot_bringup")
    robot_moveit_config_share = FindPackageShare("dock_robot_moveit_config").find("dock_robot_moveit_config")

    # ==============================
    # 配置文件路径
    # ==============================
    controllers_yaml = os.path.join(
        robot_bringup_share,
        "config",
        "dock_robot_controllers.yaml"
    )

    rviz_config = os.path.join(
        robot_moveit_config_share,
        "config",
        "mtc_solution.rviz"
    )

    # ==============================
    # MoveIt 配置构建
    # ==============================
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="dock_robot_moveit_config")
        .robot_description(file_path="config/my_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_all_params = moveit_config.to_dict()

    # ==============================
    # 1. robot_state_publisher
    # ==============================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
        ],
    )

    # ==============================
    # 2. ros2_control_node
    # ==============================
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_all_params,
            controllers_yaml,
            {"use_sim_time": False},
        ],
        additional_env={
            "LC_NUMERIC": "C",
        },
    )

    # ==============================
    # 3. 控制器加载器
    # ==============================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ==============================
    # 4. move_group
    # ==============================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
            # 关键：给 MTC 的 Exec 按钮提供 execute_task_solution action server
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    # ==============================
    # 5. RViz2
    # ==============================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
        ],
        additional_env={
            "LC_NUMERIC": "C",
            "QT_QPA_PLATFORM": "xcb",
            "RVIZ_DISABLE_LIBINPUT": "1",
        },
    )

    # ==============================
    # 6. MTC alternative_path_costs
    # ==============================
    alternative_path_costs_node = Node(
        package="dock_robot_mtc_pick_place",
        executable="alternative_path_costs",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
        ],
    )

    mtc_task_node = Node(
        package="dock_robot_mtc_pick_place",
        executable="mtc_task_node",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
        ],
    )

    # ==============================
    # 启动顺序
    # ==============================
    return LaunchDescription([
        # 先启动 robot_description 和 ros2_control
        robot_state_publisher,
        ros2_control_node,

        # 先启动 joint_state_broadcaster
        joint_state_broadcaster_spawner,

        # joint_state_broadcaster 启动完成后，启动 left_arm_controller
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    left_arm_controller_spawner,
                ],
            )
        ),

        # left_arm_controller 启动完成后，启动 right_arm_controller
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_arm_controller_spawner,
                on_exit=[
                    right_arm_controller_spawner,
                ],
            )
        ),

        # right_arm_controller 启动完成后，启动 move_group
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_arm_controller_spawner,
                on_exit=[
                    move_group_node,
                ],
            )
        ),

        # move_group 进程启动后，延时启动 RViz 和 MTC
        RegisterEventHandler(
            OnProcessStart(
                target_action=move_group_node,
                on_start=[
                    # RViz 延时 3 秒启动
                    TimerAction(
                        period=3.0,
                        actions=[
                            rviz_node,
                        ],
                    ),

                    # MTC alternative_path_costs 延时 6 秒启动
                    # 让 move_group / planning_scene_monitor / controller manager 有时间初始化
                    TimerAction(
                        period=12.0,
                        actions=[
                            mtc_task_node,
                        ],
                    ),
                ],
            )
        ),
    ])