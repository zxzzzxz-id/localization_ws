// 先验 PCD 地图 -> 2D 占据栅格。
//
// 取 z∈[z_min, z_max] 的点投影到 XY 平面，发布 /map（OccupancyGrid，
// transient_local，和 map_server 同款 QoS），可选存成 .pgm/.yaml。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr int8_t kOccupied = 100;
constexpr int8_t kFree = 0;
constexpr int8_t kUnknown = -1;
}  // namespace

class PcdToOccupancyMap : public rclcpp::Node
{
public:
  PcdToOccupancyMap() : Node("pcd_to_occupancy_map")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    resolution_ = declare_parameter<double>("resolution", 0.05);
    z_min_ = declare_parameter<double>("z_min", 0.15);
    z_max_ = declare_parameter<double>("z_max", 1.5);
    min_points_ = declare_parameter<int>("min_points_per_cell", 1);
    // true: 没观测过的格子标 unknown(-1)；false: 一律当空闲(0)
    mark_unknown_ = declare_parameter<bool>("mark_unknown", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    topic_ = declare_parameter<std::string>("occupancy_topic", "/map");
    save_prefix_ = declare_parameter<std::string>("save_prefix", "");

    if (pcd_file_.empty()) {
      throw std::runtime_error("参数 pcd_file 未设置，需要指向先验 PCD 地图");
    }
    if (z_max_ <= z_min_) {
      throw std::runtime_error("z_max 必须大于 z_min");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);

    RCLCPP_INFO(get_logger(), "读取先验地图 %s ...", pcd_file_.c_str());
    build_grid();
    log_source();
    publisher_->publish(grid_);
    RCLCPP_INFO(
      get_logger(), "栅格图 %ux%u @ %.3fm，原点 (%.2f, %.2f)，已发布到 %s",
      grid_.info.width, grid_.info.height, resolution_,
      grid_.info.origin.position.x, grid_.info.origin.position.y, topic_.c_str());

    if (!save_prefix_.empty()) {
      save_map();
    }
  }

private:
  void log_source() const
  {
    // 把真正读到的文件信息打出来，避免"看的到底是哪张图"说不清。
    std::error_code ec;
    const auto resolved = std::filesystem::canonical(pcd_file_, ec);
    const std::string path = ec ? pcd_file_ : resolved.string();

    struct stat file_info {};
    std::string stamp = "未知";
    if (stat(path.c_str(), &file_info) == 0) {
      char buffer[32] = {0};
      std::strftime(buffer, sizeof(buffer), "%F %T",
                    std::localtime(&file_info.st_mtime));
      stamp = buffer;
    }
    RCLCPP_INFO(get_logger(), "先验地图实际文件: %s", path.c_str());
    RCLCPP_INFO(
      get_logger(), "  大小 %ld 字节 | 修改时间 %s | 总点数 %zu | z∈[%.2f,%.2f] 内 %zu 点",
      static_cast<long>(file_info.st_size), stamp.c_str(), total_points_, z_min_,
      z_max_, band_points_);
  }

  void build_grid()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, cloud) != 0) {
      throw std::runtime_error("读不了 PCD 文件: " + pcd_file_);
    }
    total_points_ = cloud.size();

    const double inf = std::numeric_limits<double>::max();
    double min_x = inf, min_y = inf, max_x = -inf, max_y = -inf;
    std::size_t kept = 0;
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      if (point.z < z_min_ || point.z > z_max_) {
        continue;
      }
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      ++kept;
    }
    if (kept == 0) {
      band_points_ = 0;
      throw std::runtime_error("z 高度带内没有点，检查 z_min / z_max");
    }
    band_points_ = kept;

    const double origin_x = std::floor(min_x / resolution_) * resolution_;
    const double origin_y = std::floor(min_y / resolution_) * resolution_;
    const double span_x = std::ceil(max_x / resolution_) * resolution_ - origin_x;
    const double span_y = std::ceil(max_y / resolution_) * resolution_ - origin_y;
    const int width = std::max(1, static_cast<int>(std::lround(span_x / resolution_)));
    const int height = std::max(1, static_cast<int>(std::lround(span_y / resolution_)));

    // counts: z 高度带内的点数（判占据）；seen: 该列有没有过任何点（判是否观测到）
    std::vector<int> counts(static_cast<std::size_t>(width) * height, 0);
    std::vector<int> seen(static_cast<std::size_t>(width) * height, 0);
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      const int ix = std::clamp(
        static_cast<int>(std::floor((point.x - origin_x) / resolution_)), 0, width - 1);
      const int iy = std::clamp(
        static_cast<int>(std::floor((point.y - origin_y) / resolution_)), 0, height - 1);
      const std::size_t index = static_cast<std::size_t>(iy) * width + ix;
      ++seen[index];
      if (point.z >= z_min_ && point.z <= z_max_) {
        ++counts[index];
      }
    }

    grid_.header.frame_id = frame_id_;
    grid_.header.stamp = now();
    grid_.info.resolution = resolution_;
    grid_.info.width = static_cast<uint32_t>(width);
    grid_.info.height = static_cast<uint32_t>(height);
    grid_.info.origin.position.x = origin_x;
    grid_.info.origin.position.y = origin_y;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.resize(counts.size());
    for (std::size_t i = 0; i < counts.size(); ++i) {
      if (counts[i] >= min_points_) {
        grid_.data[i] = kOccupied;
      } else if (!mark_unknown_ || seen[i] > 0) {
        grid_.data[i] = kFree;
      } else {
        grid_.data[i] = kUnknown;
      }
    }
  }

  void save_map() const
  {
    const int width = static_cast<int>(grid_.info.width);
    const int height = static_cast<int>(grid_.info.height);

    // map_server 约定 0=占据(黑)、254=空闲(白)、205=未知(灰)；图像行序自上而下，
    // 栅格行序自下而上，所以要上下翻转。
    std::vector<uint8_t> image(static_cast<std::size_t>(width) * height, 254);
    for (int row = 0; row < height; ++row) {
      const int image_row = height - 1 - row;
      for (int col = 0; col < width; ++col) {
        const int8_t value = grid_.data[static_cast<std::size_t>(row) * width + col];
        uint8_t & pixel = image[static_cast<std::size_t>(image_row) * width + col];
        if (value == kOccupied) {
          pixel = 0;
        } else if (value < 0) {
          pixel = 205;
        }
      }
    }

    const std::string pgm_path = save_prefix_ + ".pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      RCLCPP_WARN(get_logger(), "写不了 %s", pgm_path.c_str());
      return;
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<const char *>(image.data()),
              static_cast<std::streamsize>(image.size()));

    const std::string yaml_path = save_prefix_ + ".yaml";
    std::ofstream yaml(yaml_path);
    if (!yaml) {
      RCLCPP_WARN(get_logger(), "写不了 %s", yaml_path.c_str());
      return;
    }
    const std::string base = pgm_path.substr(pgm_path.find_last_of('/') + 1);
    yaml << "image: " << base << "\n"
         << "resolution: " << grid_.info.resolution << "\n"
         << "origin: [" << grid_.info.origin.position.x << ", "
         << grid_.info.origin.position.y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";

    RCLCPP_INFO(get_logger(), "已保存 %s 和 %s", pgm_path.c_str(), yaml_path.c_str());
  }

  std::string pcd_file_;
  double resolution_{0.05};
  double z_min_{0.15};
  double z_max_{1.5};
  int min_points_{1};
  bool mark_unknown_{true};
  std::string frame_id_;
  std::string topic_;
  std::string save_prefix_;
  std::size_t total_points_{0};
  std::size_t band_points_{0};

  nav_msgs::msg::OccupancyGrid grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PcdToOccupancyMap>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("pcd_to_occupancy_map"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
