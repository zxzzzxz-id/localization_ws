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
find_package(serial REQUIRED)

add_executable(tf_to_serial_node src/tf_to_serial_node.cpp)
ament_target_dependencies(tf_to_serial_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  serial
)

install(TARGETS tf_to_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
