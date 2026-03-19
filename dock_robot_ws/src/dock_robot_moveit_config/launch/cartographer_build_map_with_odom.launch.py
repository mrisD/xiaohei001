import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    packagepath = get_package_share_directory('nav_car') 

    world_sdf=packagepath+'/sdf/nav_world.sdf'

    carmodel=packagepath+'/sdf/nav_car.sdf'
    robot_desc=open(carmodel).read()
 
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    resolution = LaunchConfiguration('resolution', default='0.05')

    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(packagepath, 'config') )

    configuration_basename = LaunchConfiguration('configuration_basename', default='with_odom_2d.lua')

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
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ]
        )
    
    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py']),
            launch_arguments=[('use_sim_time','true')]
        )       
 
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[ '-d', packagepath+'/sdf/rviz.rviz', ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')


    return LaunchDescription([
        gazebo_node,
        robot_to_gazebo,
        bridge_node,
        image_bridge_node,
        robot_desc_node,
        nav_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
