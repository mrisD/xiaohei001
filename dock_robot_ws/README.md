Ubuntu22.04 ROS2(Humble) Gazebo(Fortress)  

### 安裝ros 按照提示选择安装humble（桌面版）等待安装完成  
sudo apt update  
sudo apt --upgrade  
wget http://fishros.com/install -O fishros && bash fishros  

### 安装缺少的程序
sudo apt install ros-humble-moveit  
sudo apt install ros-humble-ros2-control*  
sudo apt install ros-humble-gz-ros2-control    
sudo apt install ros-humble-controller-manager  
sudo apt install ros-humble-ros2-control  
sudo apt install ros-humble-ros2-controllers  
sudo apt install ros-humble-ros-gz  

### 安装常用的控制器
sudo apt install ros-humble-joint-state-broadcaster  
sudo apt install ros-humble-joint-trajectory-controller  
sudo apt install ros-humble-diff-drive-controller  
sudo apt install ros-humble-teleop-twist-keyboard  

### 编译
### 在src同级目录下执行colocn build 
### 编译完成后生成 build install log 三个目录
### 执行source install/setup.bash

### 启动脚本
### 启动机械臂MoveIt2/仿真  
ros2 launch dock_robot_moveit_config demo.launch.py  
### 启动机械臂MoveIt2/真实硬件    
ros2 launch dock_robot_bringup dock_robot_moveit.launch.py 
### 启动建图
ros2 launch dock_robot_moveit_config build_map_nav.launch.py  
### 启动地图导航
ros2 launch dock_robot_moveit_config self_map_nav.launch.py  
### 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard  



### HARDWARE测试 话题控制电机转动  
ros2 topic pub -1 /left_arm_controller/joint_trajectory trajectory_msgs/msg/  JointTrajectory "  
joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']  
points:  
- positions: [0.0, 0.0, 0.0, 0.0, 0.0]  
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0]  
  time_from_start:  
    sec: 1  
    nanosec: 0  
"  

ros2 topic pub -1 /right_arm_controller/joint_trajectory trajectory_msgs/msg/  JointTrajectory "  
joint_names: ['jointr1', 'jointr2', 'jointr3', 'jointr4', 'jointr5']  
points:  
- positions: [0.0, 0.0, 0.0, 0.0, 0.0]  
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0]  
  time_from_start:  
    sec: 1  
    nanosec: 0  
"  

### 仿真与硬件控制切换    
### 找到dock_robot_moveit_config    
###    -config    
###    --my_robot.ros2_control.xacro    
### 打开文件，修改<hardware>中的内容，第一第二行打开时使用模拟硬件，第三行打开时使用真实硬件      
    <hardware>    
      <!-- <plugin>mock_components/GenericSystem</plugin> -->    
      <!-- <param name="calculate_dynamics">true</param> -->    
      <plugin>arm_hardware/ArmHardwareInterface</plugin>    
    </hardware>    