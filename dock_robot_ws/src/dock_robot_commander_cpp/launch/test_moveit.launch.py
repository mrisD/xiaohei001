from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    packagepath = get_package_share_directory('dock_robot_moveit_config')
    
    # 加载moveit配置
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="dock_robot_moveit_config")
        .robot_description('config/my_robot.gazebo.urdf.xacro')
        .robot_description_semantic('config/my_robot.srdf')
        .robot_description_kinematics('config/kinematics.yaml')
        .to_moveit_configs()
    )
    
    # 测试节点
    test_moveit_node = Node(
        package="dock_robot_commander_cpp",
        executable="test_moveit",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        test_moveit_node
    ])