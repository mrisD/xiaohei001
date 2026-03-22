from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os
from launch.substitutions import LaunchConfiguration
import launch

def generate_launch_description():
    packagepath = get_package_share_directory('dock_robot_moveit_config')
    print(packagepath)

    world_path = os.path.join(
    get_package_share_directory('dock_robot_moveit_config'),
    'world',
    'nav_world.sdf'
    )

    slam_config=os.path.join(
    get_package_share_directory('dock_robot_moveit_config'),
    'config',
    'slam_toolbox.yaml'
    )

    map_file = os.path.join(
        get_package_share_directory("dock_robot_moveit_config"),  # 👈 改这里！
        "maps",                          # 地图文件夹
        "map_1725111373.yaml"                  # 你的地图文件名
    )

    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(packagepath, 'config') )

    configuration_basename = LaunchConfiguration('configuration_basename', default='with_odom_2d.lua')
    
     # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(packagepath, 'maps', 'map_1774184152.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(packagepath, 'config', 'nav2_params.yaml'))

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="dock_robot_moveit_config")
        .robot_description('config/my_robot.gazebo.urdf.xacro')
        .robot_description_semantic('config/my_robot.srdf')
        .robot_description_kinematics('config/kinematics.yaml')
        
        .planning_scene_monitor(
            publish_robot_description=True,           # 在这里设置
            publish_robot_description_semantic=True   # 在这里设置
        )
        .to_moveit_configs()
    )
    # 启动Gazebo


    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim')+'/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': f' {world_path} -r'}.items()
    )

    # 将机械臂添加到Gazebo
    robot_to_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', moveit_config.robot_description['robot_description'],
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
            # 注：原代码行尾参数不完整，这里按常见用法补全
        ]
    )


    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[ 
            {'config_file':packagepath+'/config/gazebo_bridge.yaml'},
            {'use_sim_time': True},
            # {"qos_overrides./lidar.publisher.reliability": "best_effort"},
            # {"queue_size": 100},  # 超大队列
            ],
    )

        # 发布机械臂状态
    robot_desc_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': True},  # 必须使用仿真时间
            {"publish_frequency": 30.0,},
        ],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_dir],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )


    # 启动关节状态发布器, arm组控制器, 夹抓控制器
    # controller_spawner_node = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "left_arm_controller",
    #         "right_arm_controller",
    #         "mobile_controller",
    #         "body_controller",
    #     ],
    #     parameters=[
    #     {'use_sim_time': True},],
    # )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}],
    )

    left_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller"],
        parameters=[{'use_sim_time': True}],
    )

    right_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller"],
        parameters=[{'use_sim_time': True}],
    )

    body_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["body_controller"],
        parameters=[{'use_sim_time': True}],
    )


    mobile_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mobile_controller",
            "-c", "/controller_manager",
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
    )

    # # 在你的launch.py文件中
    # mobile_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "mobile_controller",
    #         "--controller-manager", "/controller_manager",
    #         "-r", "/mobile_controller/cmd_vel:=/cmd_vel"  # 关键行：重映射话题
    #     ],
    #     parameters=[{'use_sim_time': True}],
    #     output="screen",
    # )

    # 启动move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {'use_sim_time': True}],
    )

    # 延迟启动move_group
    delayed_move_group_node = TimerAction(
        period=1.0,  # 延迟5秒
        actions=[move_group_node]
    )

     

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'),
            '/online_async_launch.py']),
        launch_arguments={
            "slam_params_file": slam_config,  
            "use_sim_time": "True",
        }.items()
    )


    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py"
            )
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path}.items(),
    )

    # nav_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('nav2_bringup'), 'launch'),
    #         '/bringup_launch.py']),
    #         launch_arguments=[('use_sim_time','True')]
    #     )    

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'),
            '/online_async_launch.py'])
        )
    
    # nav_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('nav2_bringup'), 'launch'),
    #         '/bringup_launch.py']),
    #         launch_arguments=[('use_sim_time','True')]
    #     )

    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=[
            '/cmd_vel',
            '/mobile_controller/cmd_vel_unstamped'
        ],
        output='screen'
    )
    


 



    return LaunchDescription([
        gazebo_node,
        robot_to_gazebo_node,
        bridge_node,
        robot_desc_node,
        rviz_node,
        # controller_spawner_node,

        joint_state_broadcaster_spawner,
        left_arm_spawner,
        right_arm_spawner,
        body_spawner,
        mobile_controller_spawner,   


        delayed_move_group_node,
        # slam_node,
        nav_node,
        # cartographer_node,
        # occupancy_grid_node
        relay_node
    ])



