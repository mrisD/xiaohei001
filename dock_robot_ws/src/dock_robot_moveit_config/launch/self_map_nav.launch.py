import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,TimerAction

package_dir = get_package_share_directory('nav_car')
nav2_bringup_dir = get_package_share_directory('nav2_bringup')

world_sdf=package_dir+'/sdf/nav_world.sdf'

carmodel=package_dir+'/sdf/nav_car.sdf'
robot_desc=open(carmodel).read()

map_yaml_path = LaunchConfiguration('map',default=os.path.join(package_dir,'map/slam_toolbox','map_1725111373.yaml'))

def generate_launch_description():
    ld = LaunchDescription()

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', f'{world_sdf} -r')]
        )

    robot_to_gazebo = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[ '-string', robot_desc, '-x', '1', '-y', '1', '-z','0.05' ,'-name', 'nav_car']
        )

    bridge_node=Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[ {'config_file':package_dir +'/sdf/nav_bridge.yaml'},],
        )

    image_bridge_node=Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera']
    )

    robot_desc_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
        )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[ '-d', package_dir +'/sdf/rviz.rviz', ]
    )

    nav_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': 'true',
                }.items(),
        )

    delay_action = TimerAction(
        period=5.0, 
        actions=[gazebo_node,
                robot_to_gazebo,
                bridge_node,
                image_bridge_node,
                robot_desc_node,
                nav_node]
    )

    ld.add_action(rviz_node)
    ld.add_action(delay_action)

    return ld