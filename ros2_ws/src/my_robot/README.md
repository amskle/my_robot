# my_robot —— ROS 2 机器人建模与仿真实验

---

## 实验环境

| 软件 | 版本 |
|------|------|
| Ubuntu | 22.04 LTS（推荐，运行于 VirtualBox）|
| ROS 2 | Humble Hawksbill |
| Gazebo | Gazebo Classic 11 |
| CMake | ≥ 3.8 |

---

## 功能包结构

```
my_robot/
├── CMakeLists.txt           # 构建配置
├── package.xml              # 包描述与依赖
├── urdf/
│   └── robot.urdf.xacro    # 机器人模型（底盘/摄像头/激光/IMU/差速轮/随动轮）
├── worlds/
│   └── my_world.world      # Gazebo 仿真世界（房间 + 桌椅 + 障碍物）
├── config/
│   └── ros2_controllers.yaml  # ros2_control 控制器参数
├── launch/
│   ├── sim.launch.py        # 一键启动仿真
│   └── display.launch.py    # 启动 rviz2 + rqt
├── rviz/
│   └── robot.rviz           # rviz2 预置配置
└── src/
    ├── robot_teleop.cpp     # 键盘遥控节点（C++）
    ├── sensor_monitor.cpp   # 传感器监控节点（C++）
    └── auto_navigator.cpp   # 自动避障演示节点（C++）
```

---

## 安装依赖

```bash
# 更新 apt
sudo apt update

# ROS 2 Humble（若未安装，参考 https://docs.ros.org/en/humble/Installation.html）
# 以下安装本包额外依赖
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins
```

---

## 编译

```bash
# 进入工作区（示例路径 ~/ros2_ws）
mkdir -p ~/ros2_ws/src
cp -r my_robot ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot
source install/setup.bash
```

---

## 运行步骤

### 步骤 1：启动 Gazebo 仿真

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot sim.launch.py
```

等待约 10~30 秒（VirtualBox 中 Gazebo 加载较慢），直到出现机器人模型。

### 步骤 2：启动可视化工具（新终端）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot display.launch.py
```

打开 rviz2 后可看到：
- **RobotModel**：机器人三维模型
- **TF**：坐标系树（odom → base_footprint → base_link → 各传感器）
- **LaserScan**：激光雷达点云（红色球）
- **Odometry**：里程计轨迹（箭头）
- **Camera**：摄像头画面

### 步骤 3a：键盘遥控（新终端）

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot robot_teleop --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped -p use_sim_time:=true
```

按键说明：
- `w/s` 前进/后退，`a/d` 左转/右转
- `空格` 紧急停止，`Ctrl+C` 退出

### 步骤 3b：传感器监控（新终端）

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot sensor_monitor
```

每秒打印一次 IMU 姿态、激光雷达距离统计、里程计位置，
并发布 `/diagnostics_text` 供 rqt Topic Monitor 显示。

### 步骤 3c：自动避障演示（新终端）

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot auto_navigator
```

机器人将自主在仿真环境中巡行，遇到障碍自动旋转绕行。

---

## 关键参数说明

### 机器人模型参数（urdf/robot.urdf.xacro）

| 参数 | 值 | 说明 |
|------|----|------|
| base_length | 0.50 m | 底盘长度 |
| base_width  | 0.30 m | 底盘宽度 |
| base_height | 0.12 m | 底盘高度 |
| wheel_radius | 0.08 m | 驱动轮半径 |
| wheel_width  | 0.04 m | 驱动轮宽度 |
| caster_radius | 0.03 m | 随动轮半径 |
| lidar 射线数 | 360 | 每圈扫描点 |
| lidar 最大测距 | 10.0 m | |
| 摄像头分辨率 | 640×480 | 30 Hz |
| IMU 更新频率 | 100 Hz | |

### ros2_control 控制器参数（config/ros2_controllers.yaml）

| 参数 | 值 | 说明 |
|------|----|------|
| wheel_separation | 0.35 m | 两轮轴距 |
| wheel_radius | 0.08 m | 驱动轮半径 |
| max_velocity (linear) | 1.0 m/s | 最大线速度 |
| max_velocity (angular) | 2.0 rad/s | 最大角速度 |
| cmd_vel_timeout | 0.5 s | 超时自动停车 |

### 自动导航参数（可通过 ROS 参数覆盖）

```bash
ros2 run my_robot auto_navigator --ros-args \
  -p linear_speed:=0.4 \
  -p angular_speed:=1.0 \
  -p stop_dist:=0.6 \
  -p warn_dist:=1.0
```

---

## rqt 常用操作

打开 rqt 后，通过菜单选择插件：

- **Plugins → Topics → Topic Monitor**：查看所有话题数据
- **Plugins → Introspection → Node Graph**：查看节点拓扑图
- **Plugins → Robot Tools → TF Tree**：查看 TF 树结构
- **Plugins → Visualization → Plot**：实时绘制传感器数据曲线

---

## 常见问题

**Q: Gazebo 启动很慢或卡住**  
A: VirtualBox 中建议分配 ≥ 4 GB 内存，启用 3D 加速。

**Q: 控制器加载失败**  
A: 检查 `ros2 control list_hardware_interfaces`，确认 URDF 中的 ros2_control 标签正确加载。

**Q: 激光雷达话题无数据**  
A: 确认 Gazebo 插件 `libgazebo_ros_ray_sensor.so` 已正确安装，运行 `ros2 topic list | grep scan`。
