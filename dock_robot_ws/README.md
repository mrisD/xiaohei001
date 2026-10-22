# dock_arm_wheel_robot

差速轮式底盘 + 升降柱 + 双臂 5 自由度的**移动操作机器人**（mobile manipulation）ROS 2 工作区。

- ROS 2 **Humble** / Ubuntu 22.04
- 运动规划：MoveIt 2 + MoveIt Task Constructor（MTC）
- 仿真：Gazebo Fortress（`ros_gz`）
- 真机：机械臂经 **ZLG USBCAN-II** 走私有 CAN 协议；底盘经串口由 WHEELTEC 驱动包控制

```
                     head_link
                    /         \
        left_arm_link          right_arm_link
             │                      │
      joint1..joint5          jointr1..jointr5      ← 每臂 5 DOF，位置控制
             │                      │
        link5(夹爪)            linkr5(无夹爪)

   lift_link  ──(base_lift_joint, 行程 0~2.2m)── base_link ── base_footprint
                                                    │
                                          lf_wheel_link / rf_wheel_link
```

---

## 目录

- [1. 环境安装](#1-环境安装)
- [2. 包结构](#2-包结构)
- [3. 关节 / 控制器 / 电机对照](#3-关节--控制器--电机对照)
- [4. 编译](#4-编译)
- [5. 运行](#5-运行)
- [6. 仿真 ↔ 真机切换](#6-仿真--真机切换)
- [7. 硬件准备（udev / 权限）](#7-硬件准备udev--权限)
- [8. 排查手册](#8-排查手册)
- [9. 已知问题](#9-已知问题)

---

## 1. 环境安装

### ROS 2 Humble

```bash
sudo apt update && sudo apt --upgrade
wget http://fishros.com/install -O fishros && bash fishros    # 选「桌面版」
```

### 依赖包

```bash
# MoveIt / ros2_control
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-gz-ros2-control ros-humble-ros-gz
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-diff-drive-controller

# 模型 / 可视化
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher ros-humble-xacro
sudo apt install ros-humble-tf2-tools ros-humble-rviz2

# 定位 / 导航
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# MoveIt Task Constructor
sudo apt install ros-humble-moveit-task-constructor-core ros-humble-moveit-task-constructor-msgs

# 其他
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install libpcap-dev          # lslidar 网口模式依赖
```

### 更换 DDS（可选，多机通信更稳）

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
echo $RMW_IMPLEMENTATION
```

---

## 2. 包结构

自研包（本项目核心，8 个）：

| 包 | 职责 |
|---|---|
| `dock_robot_description` | URDF / xacro 模型、STL 网格、RViz 配置 |
| `dock_robot_moveit_config` | **配置中枢**：SRDF、运动学、关节限位、ros2_control xacro、控制器 yaml、nav2/slam 参数、地图、world、大部分 launch |
| `dock_robot_hardware` | ros2_control `SystemInterface` 插件 + CAN 驱动 + `can_probe` 诊断工具 |
| `dock_robot_bringup` | 真机机械臂启动（ros2_control + MoveIt + RViz） |
| `dock_robot_gazebo_bringup` | 纯 Gazebo 起模型 + bridge |
| `dock_robot_interfaces` | 自定义消息 `PoseCommand`（x y z roll pitch yaw cartesian_path） |
| `dock_robot_commander_cpp` | 话题级控制 API（关节数组 / 位姿指令） |
| `dock_robot_mtc_pick_place` | MTC 抓取放置任务 + 多代价策略对比 demo |

厂商包（`src/s100_diff/`，WHEELTEC S100 差速底盘整套）：`turn_on_wheeltec_robot`（底盘串口驱动 + IMU + EKF）、`wheeltec_lidar_ros2`（镭神 LSLidar）、`wheeltec_robot_nav2`、`wheeltec_robot_slam`、`wheeltec_robot_urdf`、`wheeltec_robot_keyboard`、`wheeltec_robot_msg`、`nav2_waypoint_cycle`、`interfaces`、`depend/serial_ros2`。

> 底盘相关的唯一需要改的地方是 `turn_on_wheeltec_robot/config/wheeltec_param.yaml`：
> `car_mode: S100_diff`、`lidar_type:`、`imu_mode: stm32`。

---

## 3. 关节 / 控制器 / 电机对照

| 关节 | SRDF 组 | 控制器 | 电机 CAN ID | CAN 通道 |
|---|---|---|---|---|
| `joint1`~`joint5` | `left_arm` | `left_arm_controller` | `0x01 0x02 0x03 0x04 0x0A` | **CAN1** |
| `jointr1`~`jointr5` | `right_arm` | `right_arm_controller` | `0x06 0x07 0x08 0x09 0x05` | **CAN2** |
| `lf_wheel_joint` / `rf_wheel_joint` | `mobile` | `mobile_controller`（diff_drive） | — | 无（真机归 `turn_on_wheeltec_robot`） |
| `base_lift_joint` | `body` | `body_controller` | — | 无（未接硬件） |
| `gripper_left_finger_joint` / `right` | `left_gripper` | — | — | 无（仅视觉，未接 ros2_control） |

CAN 参数（`can_driver.hpp` / `can_driver.cpp`）：

- 设备 `VCI_USBCAN2`，SJA1000 16 MHz
- 当前配置 `Timing0=0x00, Timing1=0x14` → **1 Mbps**（注意源码注释写的是 500 kbps，与值不符；500 kbps 应为 `0x1C`）
- 电机参数：减速比 `101`，编码器 `65536` 计数/圈
- 协议：`0x1E + int32 计数` = 设定目标角度；`0x08` = 读位置，回帧 `Data[0]==0x08 + int32 计数`

---

## 4. 编译

```bash
cd ~/Documents/dock_arm_wheel_robot/dock_robot_ws
rosdep install --from-paths src --ignore-src -r -y     # 可选
colcon build --symlink-install
source install/setup.bash
```

### ⚠️ 强烈建议加 `--symlink-install`

`ros2 launch` 读的是 **`install/`**，不是 `src/`。`dock_robot_moveit_config` 用 `install(DIRECTORY config ...)` 在**构建时复制**配置文件，所以：

> 改了 `src/.../config/*.xacro|yaml|srdf` 之后，**不重新 `colcon build` 就不会生效**。

首次就用 `--symlink-install` 建一次工作区，之后改配置文件就能直接重启 launch 生效，不用反复编译。

单独编某个包：

```bash
colcon build --packages-select dock_robot_hardware --symlink-install
```

---

## 5. 运行

| 场景 | 命令 |
|---|---|
| 只看模型（滑块调关节） | `ros2 launch dock_robot_description display.launch.xml` |
| MoveIt 纯仿真 | `ros2 launch dock_robot_moveit_config demo.launch.py` |
| Gazebo 仿真 + MoveIt | `ros2 launch dock_robot_moveit_config gazebo.launch.py` |
| **真机机械臂** | `ros2 launch dock_robot_bringup dock_robot_moveit.launch.py` |
| 仿真建图 | `ros2 launch dock_robot_moveit_config build_map_nav.launch.py` |
| 仿真地图导航 | `ros2 launch dock_robot_moveit_config self_map_nav.launch.py` |
| MTC 抓取放置 | `ros2 launch dock_robot_mtc_pick_place mtc_demos.launch.py` |
| 纯 Gazebo 起模型 | `ros2 launch dock_robot_gazebo_bringup dock_robot_gazebo.launch.xml` |

### 5.1 真机机械臂

```bash
# ① 先切硬件插件（见第 6 节），确认 config/my_robot.ros2_control.xacro 用的是 ArmHardwareInterface
# ② 启动
ros2 launch dock_robot_bringup dock_robot_moveit.launch.py
```

启动成功的标志（`ros2_control_node` 的输出里）：

```
CAN_DRIVER initialized
CAN 初始化成功+++++++++++++++  CAN1=左臂，CAN2=右臂
读取初始位置成功: joint1, motor_id=0x01, ...      ← 应该有 10 行
全部电机初始位置读取成功，硬件插件激活完成
```

**如果看到 `有 N 个电机初始位置读取失败`，机械臂不会动，但系统其他部分全部正常** —— 这是设计上故意的（硬件故障不阻断 ros2_control）。请直接看 [8.1](#81-真机不动--电机无反馈)。

### 5.2 真机底盘

```bash
# 底盘（串口驱动 + EKF + 底盘 URDF）
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 底盘 + 雷达 + 相机
ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py

# 键盘遥控
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
```

整机联调通常开两个终端：

```bash
# 终端 1：底盘 + 传感器
ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py
# 终端 2：机械臂
ros2 launch dock_robot_bringup dock_robot_moveit.launch.py
```

> ⚠️ 真机上**不要** spawn `mobile_controller`。底盘由 `turn_on_wheeltec_robot` 驱动并发布自己的里程计，`mobile_controller` 会和它抢 `/cmd_vel` 并产生第二套 odom/TF。`dock_robot_bringup` 的 launch 已经正确地没有 spawn 它。

### 5.3 控制 API

```bash
ros2 run dock_robot_commander_cpp commander
```

它**只绑定了 `right_arm`**，两个订阅：

```bash
# 关节目标（Float64MultiArray，长度必须正好 5）
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"

# 位姿目标（坐标系硬编码 base_link）
ros2 topic pub --once /pose_command dock_robot_interfaces/msg/PoseCommand \
  "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```

绕过 MoveIt，直接往控制器发轨迹（**调试单个关节时最方便**）：

```bash
ros2 topic pub --once /left_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "
joint_names: ['joint1','joint2','joint3','joint4','joint5']
points:
- positions: [0.05, 0.0, 0.0, 0.0, 0.0]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
  time_from_start: {sec: 3, nanosec: 0}
"
```

> 真机首次调试务必**一次只动一个关节、0.05 rad 起、把 `joint_limits.yaml` 的 `max_velocity` 临时压到 0.1**，逐个确认转向与 RViz 一致。硬件层没有任何限幅保护。

### 5.4 MTC 抓取放置

```bash
ros2 launch dock_robot_mtc_pick_place mtc_demos.launch.py
```

- `mtc_task_node`：左臂 + 左夹爪的完整 pick & place，默认 `execute=false`（只规划，在 RViz 的 Motion Planning Tasks 面板看解）
- `alternative_path_costs`：同一条 `left_arm_home → left_arm_pose1` 路径用 4 种代价（`PathLength` / `TrajectoryDuration` / `LinkMotion("link5")` / `LinkMotion("link4")`）各算一遍，推给 RViz 对比

要真正执行需要在 `mtc_demos.launch.py` 里给 `mtc_task_node` 加参数，注意还要覆盖两个过时的默认值（见 [9. 已知问题](#9-已知问题)）：

```python
parameters=[moveit_all_params, {"use_sim_time": False},
            {"execute": True},
            {"arm_home_pose": "left_arm_home"},
            {"controller_names": ["left_arm_controller"]}]
```

### 5.5 建图 / 导航

仿真：`build_map_nav.launch.py` 用 cartographer 建图（world = `world/nav_world.sdf`），`self_map_nav.launch.py` 用已有地图跑 nav2。

```bash
# 仿真建图
ros2 launch dock_robot_moveit_config build_map_nav.launch.py
# 仿真导航（默认地图 maps/map_007.yaml）
ros2 launch dock_robot_moveit_config self_map_nav.launch.py
# 换地图
ros2 launch dock_robot_moveit_config self_map_nav.launch.py \
  map:=$(ros2 pkg prefix dock_robot_moveit_config)/share/dock_robot_moveit_config/maps/map_1774184152.yaml
```

真机建图 / 导航走 WHEELTEC 那一套：

```bash
ros2 launch wheeltec_slam_toolbox online_async_launch.py     # 建图
ros2 launch wheeltec_nav2 save_map.launch.py                 # 存图
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py            # 导航
```

已保存地图：`dock_robot_moveit_config/maps/` 下 `map_007`、`map_1725111373`、`map_1773903490`、`map_1774184152`。

保存 cartographer 地图：

```bash
bash scripts/save_cartographer_map.sh     # 注意脚本里的 MAP_PATH 是硬编码的，先改
```

---

## 6. 仿真 ↔ 真机切换

只改一个文件：`dock_robot_moveit_config/config/my_robot.ros2_control.xacro`。

**仿真（默认）：**

```xml
<hardware>
    <plugin>mock_components/GenericSystem</plugin>
    <param name="calculate_dynamics">true</param>
    <!-- <plugin>arm_hardware/ArmHardwareInterface</plugin> -->
</hardware>
```

**真机：**

```xml
<hardware>
    <!-- <plugin>mock_components/GenericSystem</plugin> -->
    <!-- <param name="calculate_dynamics">true</param> -->
    <plugin>arm_hardware/ArmHardwareInterface</plugin>
</hardware>
```

注意目录下有两个名字相近的文件，别改错：

| 文件 | 用于 |
|---|---|
| `config/my_robot.ros2_control.xacro` | `demo.launch.py`、**`dock_robot_bringup/dock_robot_moveit.launch.py`（真机）** |
| `config/my_robot.gazebo.ros2_control.xacro` | `gazebo.launch.py`、`build_map_nav.launch.py`、`self_map_nav.launch.py` |

改完按第 4 节的说法让改动生效（`--symlink-install` 则直接重启 launch）。

**验证运行时到底加载了哪个插件**（不要靠看文件）：

```bash
ros2 control list_hardware_components
#   期望: plugin name: arm_hardware/ArmHardwareInterface
#         state: id=3 label=active
ros2 control list_controllers
#   期望: joint_state_broadcaster / left_arm_controller / right_arm_controller 均 active
```

---

## 7. 硬件准备（udev / 权限）

### 7.1 USBCAN-II（机械臂）

```bash
sudo tee /etc/udev/rules.d/99-myusb.rules >/dev/null <<'EOF'
ACTION=="add",SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", GROUP="users", MODE="0777"
EOF
# 插拔 USBCAN 或重启
```

确认设备被识别：

```bash
lsusb | grep -i 04d8         # 期望 04d8:0053
```

### 7.2 底盘 / 雷达串口

```bash
sudo bash src/s100_diff/turn_on_wheeltec_robot/wheeltec_udev.sh
sudo udevadm control --reload && sudo udevadm trigger
```

该脚本按 USB 串口**序号**建符号链接：

| 符号链接 | 序号 | 用途 |
|---|---|---|
| `/dev/wheeltec_controller` | `0002` | 底盘 STM32（115200） |
| `/dev/wheeltec_lidar` | `0001` | 雷达（N10Plus：460800） |

换过 USB 口或换过板子后序号可能变，需要改脚本里的 `ATTRS{serial}`：

```bash
udevadm info -a -n /dev/ttyACM1 | grep -iE 'serial|idVendor|idProduct'
```

### 7.3 实时调度（可选）

启动时会看到 `Could not enable FIFO RT scheduling policy: Operation not permitted`，不影响功能。要让 500 Hz 控制更稳：

```bash
echo '@users - rtprio 99'   | sudo tee -a /etc/security/limits.conf
echo '@users - memlock unlimited' | sudo tee -a /etc/security/limits.conf
# 重新登录生效
```

---

## 8. 排查手册

### 8.1 真机不动 / 电机无反馈

**症状**：`ros2 launch` 一切正常、控制器 active、MoveIt 能规划，但机械臂不动；RViz 里机器人冻在全 0 位姿。

**原因**：`on_activate()` 读不到任何电机初始位置。设计上 CAN 故障**不会**让 `ros2_control_node` 退出，所以表面看不出来。

先在 launch 终端里找关键行：

| 终端输出 | 含义 |
|---|---|
| `VCI_OpenDevice 失败` / `CAN设备打开失败，硬件未连接` | USBCAN 没插、被占用、没权限 → 查 [7.1](#71-usbcan-ii机械臂) |
| `有 N 个电机初始位置读取失败` + `未收到电机反馈` | **CAN 通了但没有电机应答** → 见下方顺序排查 |
| `发送失败：motor_id=0xXX channel=x` | 帧都没发出去 |
| 有 10 行 `读取初始位置成功` | 硬件层正常，问题在别处 |

排查顺序（按命中率）：

1. **CAN1 / CAN2 接反** ← 最常见的坑
   左臂在 CAN1、右臂在 CAN2。两个插头互换时：左臂 5 个 ID 的查询帧发到右臂总线上，右臂控制器按 ID 过滤后全部忽略，反之亦然 → **恰好是「10 个全无应答」**。
2. **电机电源没开**（机械臂通常独立 24V/48V 供电）
3. **CAN_H / CAN_L 接反**、未共地
4. **120 Ω 终端电阻**缺失
5. **波特率不匹配**（见第 3 节，当前 1 Mbps）

**用 `can_probe` 一次性定位**（先 Ctrl+C 停掉 `ros2 launch`，否则 USB 设备被占用）：

```bash
colcon build --packages-select dock_robot_hardware --symlink-install
source install/setup.bash

# ① 自发自收自检：证明 USBCAN 盒子 + libcontrolcan 本身是好的
ros2 run dock_robot_hardware can_probe --loopback

# ② 嗅探：发 0x08 查询，把 CAN1/CAN2 收到的每一帧原样打印
ros2 run dock_robot_hardware can_probe --baud 1000 --seconds 5
ros2 run dock_robot_hardware can_probe --baud 500  --seconds 5    # 试其它波特率
```

读结果：

| 输出 | 结论 |
|---|---|
| `--loopback` 失败 | USBCAN 盒子 / 驱动库 / 权限问题 |
| `总收帧数 > 0`，出现 ID `0x01`~`0x0A` | **总线通了**，就是当前波特率；`[CAN1/左臂]` / `[CAN2/右臂]` 标签能看出哪个通道有帧 |
| 有帧但首帧 `Data` 第一个字节不是 `08` 或 `DLC<5` | 电机回帧格式与 `parse_can_message()` 的假设不符，需改解析代码 |
| 两个通道 `总收帧数 = 0` | 物理层问题：电源 / 接线 / 终端电阻 / 通道 |

> `parse_can_message()` 只记录 `DataLen>=5 && Data[0]==0x08` 的帧，**其它帧静默丢弃**。所以「没反馈」有两种可能：总线真没数据，或回了但格式不符。`can_probe` 就是为了把这两种分开。

### 8.2 改了配置但不生效

`ros2 launch` 读 `install/`，不是 `src/`。要么重新 `colcon build`，要么一开始就用 `--symlink-install` 建工作区（见第 4 节）。

快速确认 install 里是哪一份：

```bash
grep -n '<plugin>' $(ros2 pkg prefix dock_robot_moveit_config)/share/dock_robot_moveit_config/config/my_robot.ros2_control.xacro
```

### 8.3 雷达刷 `CRC check failed`

`lslidar_driver_node` 反复打印：

```
CRC check failed (lowercase): calculated: 0L, received: 0Z
Abandon the current data packet.
```

三个要点：

1. **日志里的 `0L`/`0Z` 不是十六进制值**。源码把 `uint8_t` 直接喂给 `std::ostream`，是按**字符**打印的，`std::hex` 不生效：`0L` 真实值是 `0x4C`，`0Z` 是 `0x5A`。
2. **校验算法**：N10Plus 一帧 108 字节，前 107 字节累加取低 8 位应等于第 108 字节。
3. **按报错频率判断根因**：
   - 每秒**上千条** → 帧头 `A5 5A` 能稳定对齐，帧长/型号不对（改 `wheeltec_param.yaml` 的 `lidar_type`）
   - **分钟级几十条** → 字节流本身是垃圾，**波特率不对**或接错设备

`lidar_type` → 驱动的映射（`wheeltec_param.yaml`）：

| `lidar_type` | 驱动内 `lidar_model` | 波特率 | 帧长 |
|---|---|---|---|
| `ls_N10Plus_uart` | `N10Plus` | 460800 | 108 |
| `ls_N10_uart` | `N10` | 230400 | 58 |
| `ls_M10_uart` | `M10` | 460800 | 92 |
| `ls_M10P_uart` | `M10P` | 500000 | 140 |

> 波特率在驱动里是**硬编码**的（`lslidar_x10_driver.cpp::configureParameters()`），不是 ROS 参数，yaml 改不了；要改必须改代码重新编译。

如果不确定是哪个型号，直接读串口原始字节看帧头间隔：

```bash
# 先停掉驱动（串口被占用）
sudo stty -F /dev/wheeltec_lidar raw 460800 -echo
timeout 2 cat /dev/wheeltec_lidar | hexdump -C | head -40
# 找 a5 5a 的间隔：稳定 108 字节 → N10Plus；稳定 58 → N10；随机 → 波特率不对
```

### 8.4 RViz 崩溃 / 打不开

真机 launch 已经设了规避变量：

```python
additional_env={"LC_NUMERIC": "C", "QT_QPA_PLATFORM": "xcb", "RVIZ_DISABLE_LIBINPUT": "1"}
```

手工启动时也可以照抄：

```bash
LC_NUMERIC=C QT_QPA_PLATFORM=xcb RVIZ_DISABLE_LIBINPUT=1 rviz2 -d <config>
```

### 8.5 底盘和机械臂争 TF / odom

真机上同时跑 `turn_on_wheeltec_robot`（发布 `odom_combined` + EKF）和 MoveIt 那套（`mobile_controller` 发布 `odom`）会出现**两套里程计、两套 TF**。

- 真机：只保留 `turn_on_wheeltec_robot`，**不要** spawn `mobile_controller`
- 仿真：反过来，只保留 `mobile_controller`，用 `topic_tools/relay` 把 `/cmd_vel` → `/mobile_controller/cmd_vel_unstamped`（`self_map_nav.launch.py` 里已有）

### 8.6 机械臂能规划但执行失败

- MoveIt 报找不到控制器 / `Unable to identify any set of controllers`：核对 `config/moveit_controllers.yaml` 里的 `controller_names` 与 `ros2 control list_controllers` 的实际名称
- 用 `body` / `mobile` 组做规划会失败：`moveit_controllers.yaml` 里 `mobile_controller` 写的关节名 `base_left_wheel_joint`/`base_right_wheel_joint` **在当前 URDF 里不存在**（实际是 `lf_wheel_joint`/`rf_wheel_joint`），且这两个控制器在真机 launch 里没有 spawn。只规划 `left_arm` / `right_arm` 组即可。

---

## 9. 已知问题

按严重度排列，都是代码里真实存在、尚未修的：

| # | 问题 | 位置 |
|---|---|---|
| 1 | `default_velocity_scaling_factor: 0.52.1` 是**非法浮点数**（YAML 会解析成字符串）。OMPL 不读它所以暂时没炸，但 `pilz_industrial_motion_planner` 会读，用 LIN/PTP/CIRC 规划时会踩 | `config/joint_limits.yaml:5-6` |
| 2 | 硬件故障静默降级：CAN 打不开、电机不同步均返回 `SUCCESS`；`read()` 无超时/失效检测（`MotorState.timestamp` 记了但从未用于判超时）→ CAN 中途掉线表现为「机器人僵住、系统一切正常」 | `src/arm_hardware_interface.cpp` |
| 3 | `read()`/`write()` 只处理 10 个臂关节；`lf/rf_wheel_joint`、`base_lift_joint` 永远停在初值 | 同上 |
| 4 | MTC 默认 `controller_names = {"arm_controller","grip_action_controller"}` 已过时；默认 `arm_home_pose = "home"` 但 SRDF 里只有 `left_arm_home` → 直接跑 `mtc_demos.launch.py` 会在建任务阶段抛 `InitStageException` | `src/mtc_task_node.cpp:115,123` |
| 5 | 夹爪没有任何执行通路：SRDF 有 `left_gripper` 组、MTC 会驱动它，但 ros2_control 未声明手指关节、也没有夹爪控制器 → 夹爪不会动 | `config/my_robot.ros2_control.xacro` |
| 6 | Gazebo 路径的 ros2_control 用了不存在的关节名 `base_left_wheel_joint`/`base_right_wheel_joint`（URDF 里是 `lf/rf_wheel_joint`），且 `joint_limits.yaml` 里同样缺少 `lf/rf_wheel_joint` 的条目 | `config/my_robot.gazebo.ros2_control.xacro:82-95`、`config/joint_limits.yaml` |
| 7 | mock 路径轮子给的是 `position` 命令接口，Gazebo 路径给的是 `velocity`；Humble 的 `diff_drive_controller` 默认要 velocity，所以 mock 路径下 `mobile_controller` 大概率加载不起来（当前只是没 spawn 才没暴露） | 两个 ros2_control xacro |
| 8 | CAN 驱动注释与值不符：注释 `// 500kbps`，实际 `Timing1=0x14` = **1 Mbps**（500 kbps 应为 `0x1C`） | `src/can_driver.cpp:83-86` |
| 9 | `on_activate()` 与接收线程并发调用 `update_states()` → 对同一 USBCAN 句柄并发 `VCI_Transmit`，ZLG 库不保证线程安全 | `src/arm_hardware_interface.cpp:93`、`src/can_driver.cpp:207` |
| 10 | `lslidar_driver` 的 `poll()` 失败后主循环无延时（`while (ok) driver->poll();`），CRC 全失败时会 100% 占满一个核；`LSIOSR::read()` 返回 -1 时会出现 `&packet->data[-1]` 越界写 | `src/s100_diff/.../lslidar_driver_node.cpp:73-77`、`lsiosr.cpp:207` |
| 11 | `dock_robot_moveit_config/CMakeLists.txt` 在 `install()` 之前调用了 `ament_package()`，后面的 install 规则不保证注册 | `CMakeLists.txt` |
| 12 | `dock_robot_interfaces/CMakeLists.txt` 重复 `find_package(rosidl_default_generators)` | `CMakeLists.txt` |
| 13 | 仓库卫生：`my_robot copy.srdf`、`my_robot.srdf.backup`、`include/.../mtc_task_node.bak`、`__pycache__/*.pyc`、历史地图都在版本库里 | `src/dock_robot_moveit_config/`、`src/dock_robot_mtc_pick_place/` |

---

## 附录：深度相机（pyorbbecsdk）

```bash
pip install pyorbbecsdk2-2.0.18-cp310-cp310-linux_aarch64.whl
pip install utils
```
