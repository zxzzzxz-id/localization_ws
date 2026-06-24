cmake_minimum_required(VERSION 3.8)
project(tf_to_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(tf_to_serial_node src/tf_to_serial_node.cpp)

# 让 ROS2 接管所有的头文件和库链接工作！
ament_target_dependencies(tf_to_serial_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  serial
)

# IMU 重力对齐标定工具
add_executable(imu_gravity_calibration src/imu_gravity_calibration.cpp)
ament_target_dependencies(imu_gravity_calibration
  rclcpp
  sensor_msgs
)

# 平地移动动态倾角标定工具
add_executable(tilt_dynamic_calibration src/tilt_dynamic_calibration.cpp)
ament_target_dependencies(tilt_dynamic_calibration
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# LiDAR 到机器人中心的旋转标定工具
add_executable(lidar_robot_rotation_calibration src/lidar_robot_rotation_calibration.cpp)
ament_target_dependencies(lidar_robot_rotation_calibration
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# 直接以原地旋转时 robot XY 不动为目标的外参优化工具
add_executable(tf_to_serial_rotation_optimizer src/tf_to_serial_rotation_optimizer.cpp)
ament_target_dependencies(tf_to_serial_rotation_optimizer
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# LiDAR/SLAM XY 到机器人 XY 的直线标定工具
add_executable(lidar_robot_line_calibration src/lidar_robot_line_calibration.cpp)
ament_target_dependencies(lidar_robot_line_calibration
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

install(TARGETS
  tf_to_serial_node
  imu_gravity_calibration
  tilt_dynamic_calibration
  lidar_robot_rotation_calibration
  tf_to_serial_rotation_optimizer
  lidar_robot_line_calibration
  DESTINATION lib/${PROJECT_NAME})

ament_package()
