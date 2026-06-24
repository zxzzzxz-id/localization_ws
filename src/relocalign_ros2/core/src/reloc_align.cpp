#include "relocalign.hpp"

using FastGICP = fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
using FastVGICP = fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>;
#ifdef USE_VGICP_CUDA
using FastVGICPCuda = fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ>;
using NDTCuda = fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ>;
#endif

fast_gicp::NeighborSearchMethod search_method(const std::string& neighbor_search_method) {
  if(neighbor_search_method == "DIRECT1") 
  {
    return fast_gicp::NeighborSearchMethod::DIRECT1;
  } 
  else if (neighbor_search_method == "DIRECT7") 
  {
    return fast_gicp::NeighborSearchMethod::DIRECT7;
  } 
  else if (neighbor_search_method == "DIRECT27") 
  {
    return fast_gicp::NeighborSearchMethod::DIRECT27;
  } 
  else if (neighbor_search_method == "DIRECT_RADIUS") 
  {
    return fast_gicp::NeighborSearchMethod::DIRECT_RADIUS;
  }

  std::cerr << "[RelocAlign Error] Unknown Neighbor Search Method: " << neighbor_search_method 
            << ", Use Default Method: DIRECT1"<< std::endl;
  return fast_gicp::NeighborSearchMethod::DIRECT1;
}

RelocAlign::RelocAlign(const RelocAlignConfig& config) : config_(config) 
{
  std::cout << "[RelocAlign INFO] RelocAlign Method: " << config_.align_method << std::endl;
  switch (ToAlignMethod(config_.align_method)) 
  {
    case AlignMethod::ICP:
    {
      auto icp = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
      auto icp_config = std::dynamic_pointer_cast<PCLICPConfig>(config_.relocalign_config_);
      icp->setMaximumIterations(icp_config->max_iterations);
      icp->setMaxCorrespondenceDistance(icp_config->max_correspondence_distance);
      icp->setTransformationEpsilon(icp_config->transformation_epsilon);
      icp->setEuclideanFitnessEpsilon(icp_config->euclidean_fitness_epsilon);
      icp->setUseReciprocalCorrespondences(icp_config->use_reciprocal_correspondences);
      reg_pcl = icp;
      break;
    }

    case AlignMethod::NDT:
    {
      auto ndt = std::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
      auto ndt_config = std::dynamic_pointer_cast<PCLNDTConfig>(config_.relocalign_config_);
      ndt->setTransformationEpsilon(ndt_config->transformation_epsilon);
      ndt->setStepSize(ndt_config->step_size);
      ndt->setResolution(ndt_config->voxel_resolution);
      ndt->setMaximumIterations(ndt_config->max_iterations);
      reg_pcl = ndt;
      break;
    }

    case AlignMethod::GICP:
    {
      auto gicp = std::make_shared<FastGICP>();
      auto gicp_config = std::dynamic_pointer_cast<GICPConfig>(config_.relocalign_config_);
      gicp->setMaxCorrespondenceDistance(gicp_config->max_correspondence_distance);
      gicp->setCorrespondenceRandomness(gicp_config->k_correspondences);
      gicp->setNumThreads(gicp_config->num_threads);
      reg_gicp = gicp;
      break;
    }

    case AlignMethod::VGICP:
    {
      auto vgicp = std::make_shared<FastVGICP>();
      auto vgicp_config = std::dynamic_pointer_cast<VGICPConfig>(config_.relocalign_config_);
      vgicp->setCorrespondenceRandomness(vgicp_config->k_correspondences);
      vgicp->setResolution(vgicp_config->voxel_resolution);
      vgicp->setNeighborSearchMethod(search_method(vgicp_config->neighbor_search_method));
      vgicp->setNumThreads(vgicp_config->num_threads);
      reg_gicp = vgicp;
      break;
    }

    case AlignMethod::VGICPCUDA:
    {
      #ifdef USE_VGICP_CUDA
      auto vgicp_cuda = std::make_shared<FastVGICPCuda>();
      auto vgicpcuda_config = std::dynamic_pointer_cast<VGICPCUDAConfig>(config_.relocalign_config_);
      vgicp_cuda->setCorrespondenceRandomness(vgicpcuda_config->k_correspondences);
      vgicp_cuda->setNeighborSearchMethod(search_method(vgicpcuda_config->neighbor_search_method), 
          vgicpcuda_config->neighbor_search_radius);
      vgicp_cuda->setResolution(vgicpcuda_config->voxel_resolution);

      if(vgicpcuda_config->nearest_neighbor_method == "CPU_PARALLEL_KDTREE")
      {
        std::cout << "--- VGICPCUDA (CPU_PARALLEL_KDTREE) ---" << std::endl;
        vgicp_cuda->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
      }
      else if(vgicpcuda_config->nearest_neighbor_method == "GPU_BRUTEFORCE")
      {
        std::cout << "--- VGICPCUDA (GPU_BRUTEFORCE) ---" << std::endl;
        vgicp_cuda->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE);
      }
      else if(vgicpcuda_config->nearest_neighbor_method == "GPU_RBF_KERNEL")
      {
        std::cout << "--- VGICPCUDA (GPU_RBF_KERNEL) ---" << std::endl;
        vgicp_cuda->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
        vgicp_cuda->setKernelWidth(vgicpcuda_config->kernel_width);
      }

      reg_gicp = vgicp_cuda;
      #else
      std::cerr << "[RelocAlign Error] Please build fast_gicp with BUILD_VGICP_CUDA=ON" << std::endl;
      #endif
      break;
    }

    case AlignMethod::NDTCUDA:
    {
      #ifdef USE_VGICP_CUDA
      auto ndt_cuda = std::make_shared<NDTCuda>();
      auto ndtcuda_config = std::dynamic_pointer_cast<NDTCUDAConfig>(config_.relocalign_config_);
      ndt_cuda->setResolution(ndtcuda_config->voxel_resolution);
      ndt_cuda->setNeighborSearchMethod(search_method(ndtcuda_config->neighbor_search_method), 
          ndtcuda_config->neighbor_search_radius);

      if(ndtcuda_config->ndt_distance_mode == "D2D")
      {
        std::cout << "--- NDT_CUDA (D2D) ---" << std::endl;
        ndt_cuda->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
      }
      else if(ndtcuda_config->ndt_distance_mode == "P2D")
      {
        std::cout << "--- NDT_CUDA (P2D) ---" << std::endl;
        ndt_cuda->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
      }

      reg_gicp = ndt_cuda;
      #else
      std::cerr << "[RelocAlign Error] Please build fast_gicp with BUILD_VGICP_CUDA=ON" << std::endl;
      #endif
      break;
    }

    default:
      std::cerr << "[RelocAlign Error] Unknown Alignment Method: " << config_.align_method << std::endl;
  }
}

void RelocAlign::Align()
{
  if (!source_pcl_ || source_pcl_->empty() || !target_pcl_ || target_pcl_->empty()) 
  {
    std::cerr << "[RelocAlign Error] Source or Target cloud is empty!" << std::endl;
    return;
  }
  aligned_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Matrix4d T;

  if(reg_pcl != nullptr)
  {
    reg_pcl->setInputTarget(target_pcl_);
    reg_pcl->setInputSource(source_pcl_);
    reg_pcl->align(*aligned_pcl_);
    fitness_score_ = reg_pcl->getFitnessScore();

    T = reg_pcl->getFinalTransformation().cast<double>();
  }
  else if(reg_gicp != nullptr)
  {
    reg_gicp->clearTarget();
    reg_gicp->clearSource();
    reg_gicp->setInputTarget(target_pcl_);
    reg_gicp->setInputSource(source_pcl_);
    reg_gicp->align(*aligned_pcl_);
    fitness_score_ = reg_gicp->getFitnessScore();

    T = reg_gicp->getFinalTransformation().cast<double>();
  }
  t_ = T.block<3,1>(0,3);
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  q_ = Eigen::Quaterniond(R);  
}