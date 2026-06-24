#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "relocalign.hpp"
#include "read_configs.hpp"


class RelocAlignNode : public rclcpp::Node {
public:
    RelocAlignNode() : Node("relocalign_pub_node"), count_(0) {
        // 1. 声明并获取参数
        this->declare_parameter<std::string>("config_path", "");
        this->declare_parameter<std::string>("map_path", "");
        this->declare_parameter<std::string>("cloud_topic", "/livox/lidar");
        this->declare_parameter<bool>("is_livox_custom", false);
        this->declare_parameter<int>("frame_count", 5);

        std::string config_path = this->get_parameter("config_path").as_string();
        std::string map_path = this->get_parameter("map_path").as_string();
        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
        is_livox_custom_ = this->get_parameter("is_livox_custom").as_bool();
        frame_count_ = this->get_parameter("frame_count").as_int();

        // 2. 初始化算法库
        RelocAlignConfig config(config_path);
        relocalign_ = std::make_unique<RelocAlign>(config);
        voxelgrid_leaf_ = config.voxelgrid_leaf;

        // 3. 加载地图
        map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read map file: %s", map_path.c_str());
        }

        // 4. 创建发布者
        pub_reloc_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("reloc/cloud_align", 10);

        // 5. 订阅雷达数据
        if (is_livox_custom_) {
            sub_livox_custom_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                cloud_topic, 10, std::bind(&RelocAlignNode::livox_callback_custom, this, std::placeholders::_1));
        } else {
            sub_livox_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                cloud_topic, 10, std::bind(&RelocAlignNode::livox_callback_pc2, this, std::placeholders::_1));
        }

        accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }
private:
    // 处理 Livox 自定义格式
    void livox_callback_custom(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        auto current_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (const auto& p : msg->points) {
            current_cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        accumulate_and_align(current_cloud);
    }

    // 处理标准 PointCloud2 格式
    void livox_callback_pc2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto current_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *current_cloud);
        accumulate_and_align(current_cloud);
    }

    void accumulate_and_align(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
        if (input_cloud->empty()) return;

        if (count_ == 0) *accumulated_cloud_ = *input_cloud;
        else *accumulated_cloud_ += *input_cloud;

        count_++;

        if (count_ >= frame_count_) {
            perform_alignment();
            count_ = 0;
            accumulated_cloud_->clear();
        }
    }

void perform_alignment() {
        // 数据预处理：去零点 + 体素滤波
        auto source_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        
        // 简单去零点逻辑
        accumulated_cloud_->erase(
            std::remove_if(accumulated_cloud_->begin(), accumulated_cloud_->end(), 
            [](const pcl::PointXYZ& pt){ return pt.getVector3fMap().squaredNorm() < 0.01; }),
            accumulated_cloud_->end());

        pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(voxelgrid_leaf_, voxelgrid_leaf_, voxelgrid_leaf_);
        vg.setInputCloud(accumulated_cloud_);
        vg.filter(*source_filtered);

        // 算法对齐
        relocalign_->SourceCloudInput(source_filtered);
        relocalign_->TargetCloudInput(map_cloud_);
        relocalign_->Align();

        // 提取并发布位姿
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        relocalign_->GetTransform(t, q);
        publish_pose(t, q);
    }

void publish_pose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = t.x();
        msg.pose.pose.position.y = t.y();
        msg.pose.pose.position.z = t.z();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        pub_reloc_->publish(msg);
    }

    // 成员变量
    int frame_count_;
    int count_;
    float voxelgrid_leaf_;
    bool is_livox_custom_;
    std::unique_ptr<RelocAlign> relocalign_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_reloc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_custom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_pc2_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelocAlignNode>());
    rclcpp::shutdown();
    return 0;
}