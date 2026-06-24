#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 建议使用类封装，这是 ROS 2 的标准写法
class CloudCollector : public rclcpp::Node {
public:
    CloudCollector() : Node("collect_node") {
        // 1. 获取参数 (ROS 2 中参数必须先声明或指定默认值)
        this->declare_parameter<std::string>("pcd_path", "map.pcd");
        this->declare_parameter<std::string>("cloud_topic", "/points_raw");
        
        pcd_path_ = this->get_parameter("pcd_path").as_string();
        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Listening to topic: %s", cloud_topic.c_str());

        // 2. 创建订阅者
        // 使用 std::bind 绑定成员函数，10 是队列深度
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 10, std::bind(&CloudCollector::cloud_callback, this, std::placeholders::_1));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        
        if (pcl::io::savePCDFileBinary(pcd_path_, cloud) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved one frame with %zu points to %s", 
                        cloud.size(), pcd_path_.c_str());
        }
    }

    std::string pcd_path_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 使用 std::make_shared 实例化节点并开始循环
    rclcpp::spin(std::make_shared<CloudCollector>());
    rclcpp::shutdown();
    return 0;
}