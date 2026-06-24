#ifndef READ_CONFIGS_HPP_
#define READ_CONFIGS_HPP_

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <filesystem> 

enum class AlignMethod 
{
    ICP,
    NDT,
    GICP,
    VGICP,
    VGICPCUDA,
    NDTCUDA,
    UNKNOWN
};

inline AlignMethod ToAlignMethod(const std::string& str) {
    if (str == "ICP") return AlignMethod::ICP;
    if (str == "NDT") return AlignMethod::NDT;
    if (str == "GICP") return AlignMethod::GICP;
    if (str == "VGICP") return AlignMethod::VGICP;
    if (str == "VGICPCUDA") return AlignMethod::VGICPCUDA;
    if (str == "NDTCUDA") return AlignMethod::NDTCUDA;
    throw std::runtime_error("Unknown Align Method: " + str);
}

struct BaseConfig
{
    virtual void Load(const YAML::Node& node) = 0;
    virtual ~BaseConfig() = default;

};

struct PCLICPConfig : public BaseConfig
{
    int max_iterations;
    double max_correspondence_distance;
    double transformation_epsilon;
    double euclidean_fitness_epsilon;
    bool use_reciprocal_correspondences;

    PCLICPConfig() {}

    void Load(const YAML::Node& node) override
    {
        max_iterations = node["max_iterations"].as<int>();
        max_correspondence_distance = node["max_correspondence_distance"].as<double>();
        transformation_epsilon = node["transformation_epsilon"].as<double>();
        euclidean_fitness_epsilon = node["euclidean_fitness_epsilon"].as<double>();
        use_reciprocal_correspondences = node["use_reciprocal_correspondences"].as<bool>();
    }
};

struct PCLNDTConfig : public BaseConfig
{        
    double step_size;      
    double voxel_resolution;   
    double transformation_epsilon;
    int max_iterations;       

    PCLNDTConfig() {};

    void Load(const YAML::Node& node) override
    {
        step_size = node["step_size"].as<double>();
        transformation_epsilon = node["transformation_epsilon"].as<double>();
        voxel_resolution = node["voxel_resolution"].as<double>();
        max_iterations = node["max_iterations"].as<int>();
    }
};

struct GICPConfig : public BaseConfig
{
    int k_correspondences;
    double max_correspondence_distance;
    int num_threads;

    GICPConfig() {}

    void Load(const YAML::Node& node) override
    {
        k_correspondences = node["k_correspondences"].as<int>();
        max_correspondence_distance = node["max_correspondence_distance"].as<double>();
        num_threads = node["num_threads"].as<int>();
    }
};

struct VGICPConfig : public BaseConfig
{
    double k_correspondences;
    double voxel_resolution;
    std::string neighbor_search_method;
    int num_threads;

    VGICPConfig() {}
    void Load(const YAML::Node& node) override
    {
        k_correspondences = node["k_correspondences"].as<int>();
        voxel_resolution = node["voxel_resolution"].as<double>();
        neighbor_search_method = node["neighbor_search_method"].as<std::string>();
        num_threads = node["num_threads"].as<int>();
    }
};

struct VGICPCUDAConfig : public BaseConfig
{
    double k_correspondences;
    double voxel_resolution;
    std::string neighbor_search_method;
    double neighbor_search_radius;
    std::string nearest_neighbor_method;
    double kernel_width;

    VGICPCUDAConfig() {}

    void Load(const YAML::Node& node) override
    {
        k_correspondences = node["k_correspondences"].as<int>();
        voxel_resolution = node["voxel_resolution"].as<double>();
        neighbor_search_method = node["neighbor_search_method"].as<std::string>();
        neighbor_search_radius = node["neighbor_search_radius"].as<double>();
        nearest_neighbor_method = node["nearest_neighbor_method"].as<std::string>();
        kernel_width = node["kernel_width"].as<double>();
    }
};

struct NDTCUDAConfig : public BaseConfig
{
    double voxel_resolution;
    std::string neighbor_search_method;
    double neighbor_search_radius;
    std::string ndt_distance_mode;

    NDTCUDAConfig() {}

    void Load(const YAML::Node& node) override
    {
        voxel_resolution = node["voxel_resolution"].as<double>();
        neighbor_search_method = node["neighbor_search_method"].as<std::string>();
        neighbor_search_radius = node["neighbor_search_radius"].as<double>();
        ndt_distance_mode = node["ndt_distance_mode"].as<std::string>();
    }
};

struct RelocAlignConfig
{
    std::string align_method;
    std::shared_ptr<BaseConfig> relocalign_config_;
    float voxelgrid_leaf;

    RelocAlignConfig() {}

    RelocAlignConfig(const std::string& config_file)
    {
        std::cout << "[RelocAlignConfig INFO] config_file: " << config_file << std::endl;
        if(!std::filesystem::exists(config_file))
        {
            std::cout << "[RelocAlignConfig Error] config_file: " << config_file << " doesn't exist" << std::endl;
            return;
        }
        YAML::Node file_node = YAML::LoadFile(config_file);
        align_method = file_node["align_method"].as<std::string>();
        voxelgrid_leaf = file_node["voxelgrid_leaf"].as<float>();

        switch (ToAlignMethod(align_method)) 
        {
            case AlignMethod::ICP:
                relocalign_config_ = std::make_shared<PCLICPConfig>();
                relocalign_config_->Load(file_node["pcl_icp"]);
                break;
            case AlignMethod::NDT:
                relocalign_config_ = std::make_shared<PCLNDTConfig>();
                relocalign_config_->Load(file_node["pcl_ndt"]);
                break;
            case AlignMethod::GICP:
                relocalign_config_ = std::make_shared<GICPConfig>();
                relocalign_config_->Load(file_node["gicp"]);
                break;
            case AlignMethod::VGICP:
                relocalign_config_ = std::make_shared<VGICPConfig>();
                relocalign_config_->Load(file_node["vgicp"]);
                break;
            case AlignMethod::VGICPCUDA:
                relocalign_config_ = std::make_shared<VGICPCUDAConfig>();
                relocalign_config_->Load(file_node["vgicp_cuda"]);
                break;
            case AlignMethod::NDTCUDA:
                relocalign_config_ = std::make_shared<NDTCUDAConfig>();
                relocalign_config_->Load(file_node["ndt_cuda"]);
                break;
            default:
                std::cerr << "[RelocAlignConfig Error] Unknown align_method: " << align_method << std::endl;
        }
    }
};


#endif