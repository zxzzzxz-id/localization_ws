# --- RelocAlign/core/CMakeLists.txt ---
cmake_minimum_required(VERSION 3.16)
project(relocalign_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(relocalign_core SHARED src/reloc_align.cpp)

# 确保头文件路径正确，包括第三方库
target_include_directories(relocalign_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/fast_gicp/include>
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

# 只需要声明链接 fast_gicp，具体的编译由父目录决定
target_link_libraries(relocalign_core
  PUBLIC
    fast_gicp
    Eigen3::Eigen
    yaml-cpp
    ${PCL_LIBRARIES}
)