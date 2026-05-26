#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================
    dock_robot_bringup = FindPackageShare(
        'dock_robot_bringup'
    ).find('dock_robot_bringup')

    dock_robot_description = FindPackageShare(
        'dock_robot_description'
    ).find('dock_robot_description')

    # ============================================================
    # File paths
    # ============================================================
    controllers_config_path = os.path.join(
        dock_robot_bringup,
        'config',
        'dock_robot_controllers.yaml'
    )

    rviz_config_path = os.path.join(
        dock_robot_description,
        'rviz',
        'moveit_config.rviz'
    )

    # ============================================================
    # MoveIt config
    # 注意：下面这些路径是相对于 dock_robot_moveit_config 包的路径
    # ============================================================
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='my_robot',
            package_name='dock_robot_moveit_config'
        )
        .robot_description(
            file_path='config/my_robot.urdf.xacro'
        )
        .robot_description_semantic(
            file_path='config/my_robot.srdf'
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml'
        )
        .joint_limits(
            file_path='config/joint_limits.yaml'
        )
        .planning_pipelines(
            pipelines=[
                'ompl',
                'pilz_industrial_motion_planner'
            ]
        )
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml'
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()

    # ============================================================
    # robot_state_publisher
    # ============================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {
                'use_sim_time': False
            }
        ]
    )

    # ============================================================
    # ros2_control_node
    # Humble 里这样传 robot_description 可用，只是会有 deprecated 警告
    # ============================================================
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            controllers_config_path,
            {
                'use_sim_time': False
            }
        ]
    )

    # ============================================================
    # Controller spawners
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60'
        ],
        output='screen'
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60'
        ],
        output='screen'
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60'
        ],
        output='screen'
    )

    # ============================================================
    # body_controller 暂时屏蔽
    # ============================================================
    # body_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'body_controller',
    #         '--controller-manager',
    #         '/controller_manager',
    #         '--controller-manager-timeout',
    #         '60'
    #     ],
    #     output='screen'
    # )

    # ============================================================
    # mobile_controller 暂时屏蔽
    # ============================================================
    # mobile_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'mobile_controller',
    #         '--controller-manager',
    #         '/controller_manager',
    #         '--controller-manager-timeout',
    #         '60'
    #     ],
    #     output='screen'
    # )

    # ============================================================
    # move_group
    # ============================================================
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            {
                'use_sim_time': False
            }
        ]
    )

    # ============================================================
    # 等 move_group 真正准备好
    # 不靠固定 Timer，而是等 node / service / action 出现
    # ============================================================
    wait_for_move_group_ready = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                'echo "[launch] waiting for move_group ready..."; '
                'until ros2 node list 2>/dev/null | grep -q "^/move_group$"; do sleep 0.5; done; '
                'until ros2 service list 2>/dev/null | grep -q "/get_planning_scene"; do sleep 0.5; done; '
                'until ros2 action list 2>/dev/null | grep -q "/move_action"; do sleep 0.5; done; '
                'echo "[launch] move_group is ready, starting RViz..."'
            )
        ],
        output='screen'
    )

    # ============================================================
    # RViz2
    # 必须带 moveit_config_dict，否则 MotionPlanning 插件容易缺参数
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config_path
        ],
        parameters=[
            moveit_config_dict,
            {
                'use_sim_time': False
            }
        ]
    )

    # ============================================================
    # 启动顺序控制
    # ============================================================

    # joint_state_broadcaster 启动完成后，再启动 left_arm_controller
    start_left_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                left_arm_controller_spawner
            ]
        )
    )

    # left_arm_controller 启动完成后，再启动 right_arm_controller
    start_right_after_left = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[
                right_arm_controller_spawner
            ]
        )
    )

    # right_arm_controller 启动完成后，直接启动 move_group
    # 因为 body_controller 已经暂时屏蔽
    start_move_group_after_right = RegisterEventHandler(
        OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[
                move_group_node
            ]
        )
    )

    # move_group 进程启动后，开始等待 move_group 真正 ready
    start_wait_after_move_group_start = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[
                wait_for_move_group_ready
            ]
        )
    )

    # wait 脚本结束后，再启动 RViz
    start_rviz_after_move_group_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_move_group_ready,
            on_exit=[
                rviz_node
            ]
        )
    )

    # ============================================================
    # LaunchDescription
    # ============================================================
    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,

        # 第一个 spawner 直接启动
        # spawner 会自己等待 /controller_manager
        joint_state_broadcaster_spawner,

        # 后面的节点按顺序启动
        start_left_after_jsb,
        start_right_after_left,
        start_move_group_after_right,
        start_wait_after_move_group_start,
        start_rviz_after_move_group_ready,
    ])