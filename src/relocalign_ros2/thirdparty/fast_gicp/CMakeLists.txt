cmake_minimum_required(VERSION 3.16)
project(fast_gicp)

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_VGICP_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()


add_library(fast_gicp SHARED
  gicp/lsq_registration.cpp
  gicp/fast_gicp.cpp
  gicp/fast_gicp_st.cpp
  gicp/fast_vgicp.cpp
)
target_link_libraries(fast_gicp
  ${PCL_LIBRARIES}
)
if (OPENMP_FOUND)
    if (TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(fast_gicp OpenMP::OpenMP_CXX)
    endif ()
endif ()
target_include_directories(fast_gicp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

### CUDA ###
if(BUILD_VGICP_CUDA)
  set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
  add_definitions(-DUSE_VGICP_CUDA)

  cuda_add_library(fast_vgicp_cuda SHARED
    cuda/fast_vgicp_cuda.cu
    cuda/brute_force_knn.cu
    cuda/covariance_estimation.cu
    cuda/covariance_estimation_rbf.cu
    cuda/covariance_regularization.cu
    cuda/gaussian_voxelmap.cu
    cuda/find_voxel_correspondences.cu
    cuda/compute_derivatives.cu
    cuda/compute_mahalanobis.cu
    cuda/ndt_cuda.cu
    cuda/ndt_compute_derivatives.cu
  )
  target_include_directories(fast_vgicp_cuda PRIVATE
    include
    ../../thirdparty/Eigen
    ../../thirdparty/nvbio
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(fast_vgicp_cuda
    ${catkin_LIBRARIES}
  )
  cuda_add_cublas_to_target(fast_vgicp_cuda)

  # add vgicp_cuda to libfast_gicp
  target_sources(fast_gicp PRIVATE
    gicp/fast_vgicp_cuda.cpp
    ndt/ndt_cuda.cpp
  )
  target_link_libraries(fast_gicp
    fast_vgicp_cuda
  )
  add_dependencies(fast_gicp fast_vgicp_cuda)
endif()
