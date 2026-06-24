cmake_minimum_required(VERSION 3.16)
project(relocalign_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(RELOCALIGN_ROS2_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
set(RELOCALIGN_ROOT_DIR "${RELOCALIGN_ROS2_DIR}")

# 1. 基础依赖 (保持不变)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters)
find_package(OpenMP REQUIRED)

# 2. 处理 Thirdparty (fast_gicp) 
# 确保这里是整个工程中唯一一个 add_subdirectory(fast_gicp) 的地方
if(NOT TARGET fast_gicp)
  add_subdirectory(${RELOCALIGN_ROOT_DIR}/thirdparty/fast_gicp
                   ${CMAKE_BINARY_DIR}/fast_gicp)
endif()

# 3. 处理核心库 (Core)
if(NOT TARGET relocalign_core)
  add_subdirectory(${RELOCALIGN_ROOT_DIR}/core
                   ${CMAKE_BINARY_DIR}/core)
endif()

# 4. 可执行文件：点云采集 (保持不变)
add_executable(cloud_collect_node src/cloud_collect_node.cpp)
target_include_directories(cloud_collect_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_collect_node relocalign_core ${PCL_LIBRARIES})
ament_target_dependencies(cloud_collect_node rclcpp sensor_msgs pcl_conversions)

# 5. 可执行文件：重定位节点
add_executable(reloc_livox_node src/reloc_livox_node.cpp)
target_include_directories(reloc_livox_node PRIVATE include ${PCL_INCLUDE_DIRS})

# 修正链接顺序：先链接依赖，再使用 ament 宏
# 注意：这里删除了关键字 PRIVATE/PUBLIC，以兼容 ament_target_dependencies
target_link_libraries(reloc_livox_node 
  relocalign_core 
  fast_gicp
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

ament_target_dependencies(reloc_livox_node 
  livox_ros_driver2
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  pcl_conversions
)

# 6. 安装与包声明 (保持不变)
install(TARGETS cloud_collect_node reloc_livox_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
