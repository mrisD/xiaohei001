Ubuntu22.04 ROS2(Humble) Gazebo(Fortress)

# 安装缺少的程序
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ros2-control* 
sudo apt install ros-humble-ros-gz        # 安装 ROS 2 Humble 与 Gazebo 的桥接包
sudo apt install ros-humble-gz-ros2-control
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
# 安装常用的控制器
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-diff-drive-controller
sudo apt install ros-humble-teleop-twist-keyboard
# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard 

# 启动脚本
# 启动机械臂MoveIt2
ros2 launch dock_robot_moveit_config demo.launch.py 
# 启动建图
ros2 launch dock_robot_moveit_config build_map_nav.launch.py 
# 启动地图导航
ros2 launch dock_robot_moveit_config self_map_nav.launch.py
