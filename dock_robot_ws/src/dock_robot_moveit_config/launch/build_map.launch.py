import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

#获取当前包的安装地址
packagepath = get_package_share_directory('nav_car')  
print(packagepath)
#获取仿真环境的sdf
world_sdf=packagepath+'/sdf/nav_world.sdf'
#读取轮式机器人的sdf
carmodel=packagepath+'/sdf/nav_car.sdf'
robot_desc=open(carmodel).read()


def generate_launch_description():
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
            parameters=[ {'config_file':packagepath+'/sdf/nav_bridge.yaml'},],
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
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
        )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[ '-d', packagepath+'/sdf/rviz.rviz', ]
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'),
            '/online_async_launch.py'])
        )
    
    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py']),
            launch_arguments=[('use_sim_time','true')]
        )

    return LaunchDescription([
        gazebo_node,
        robot_to_gazebo,
        bridge_node,
        image_bridge_node,
        robot_desc_node,
        rviz_node,
        slam_node,
        nav_node,
    ])