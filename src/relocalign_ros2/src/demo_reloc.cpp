#include "relocalign.hpp"
#include "read_configs.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char *argv[])
{
    if (argc < 4) 
    {  
        std::cerr << "Please input three arguements ......\n" << std::endl;
        return -1;
    }
    std::string config_path = argv[1];
    std::string pcd1_path = argv[2];
    std::string pcd2_path = argv[3];
    

    RelocAlignConfig relocalignconfig(config_path);
    RelocAlign relocalign(relocalignconfig);

    auto source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto target_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd1_path, *source_cloud) == -1 || 
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd2_path, *target_cloud) == -1) 
    {
        PCL_ERROR("Couldn't read file ......\n");
        return -1;
    }
    source_cloud->erase(
        std::remove_if(source_cloud->begin(), source_cloud->end(), [=](const pcl::PointXYZ& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
        source_cloud->end());
    target_cloud->erase(
        std::remove_if(target_cloud->begin(), target_cloud->end(), [=](const pcl::PointXYZ& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
        target_cloud->end());

    std::cout << "Source Cloud Size: " << source_cloud->size() << std::endl;
    std::cout << "Target Cloud Size: " << target_cloud->size() << std::endl;

    float voxelgrid_leaf = relocalignconfig.voxelgrid_leaf;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(voxelgrid_leaf, voxelgrid_leaf, voxelgrid_leaf);

    auto source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vg.setInputCloud(source_cloud); // 必须设置输入
    vg.filter(*source_cloud_filtered);

    auto target_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vg.setInputCloud(target_cloud);
    vg.filter(*target_cloud);

    relocalign.SourceCloudInput(source_cloud);
    relocalign.TargetCloudInput(target_cloud);

    relocalign.Align();

    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    relocalign.GetTransform(t, q);
    printf("[RelocAlign INFO] Estimated Transformation: x=%.5f, y=%.5f, z=%.5f\n\
            qx=%.5f, qy=%.5f, qz=%.5f, qw=%.5f\n", t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());


    return 0;
}