cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 1. 键盘遥控节点
add_executable(robot_teleop src/robot_teleop.cpp)
ament_target_dependencies(robot_teleop rclcpp geometry_msgs)

# 2. 传感器监控节点
add_executable(sensor_monitor src/sensor_monitor.cpp)
ament_target_dependencies(sensor_monitor rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs)

# 3. 自动避障演示节点
add_executable(auto_navigator src/auto_navigator.cpp)
ament_target_dependencies(auto_navigator rclcpp geometry_msgs sensor_msgs std_msgs)

# 安装可执行文件
install(TARGETS
  robot_teleop
  sensor_monitor
  auto_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件与资源目录
install(DIRECTORY
  launch
  urdf
  worlds
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()