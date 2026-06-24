#ifndef RELOCALIGN_HPP_
#define RELOCALIGN_HPP_

#include "read_configs.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
  
class RelocAlign
{
  public:
    RelocAlign() {};
    RelocAlign(const RelocAlignConfig& config);

    void SourceCloudInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
      source_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
      std::cout << "Source Cloud Input Size: " << source_pcl_->size() << std::endl;
    }

    void TargetCloudInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
      target_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
      std::cout << "Target Cloud Input Size: " << target_pcl_->size() << std::endl;
    }

    void GetAlignedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
    {
      output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*aligned_pcl_));
    }

    void Align();

    void GetTransform(Eigen::Vector3d& t, Eigen::Quaterniond& q)
    {
      t = t_;
      q = q_;
    }

  private:
    std::shared_ptr<fast_gicp::LsqRegistration<pcl::PointXYZ, pcl::PointXYZ>> reg_gicp = nullptr;
    std::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> reg_pcl = nullptr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcl_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_pcl_;

    RelocAlignConfig config_;

    Eigen::Vector3d t_;
    Eigen::Quaterniond q_;
    double fitness_score_;


};

#endif