// Two-stage PCL relocalization: align a stationary multi-frame local cloud to a
// prior PCD map and publish the compatible FAST-LIO reloc pose.
//
// Pipeline:
// 1. Load prior PCD and build coarse/fine targets.
// 2. Accumulate startup clouds in base_frame while the robot is expected static.
// 3. Remove low time-consistency voxels before registration.
// 4. Coarse point-to-point ICP gets into the correct basin.
// 5. Filter source points by prior-map consistency using the coarse pose.
// 6. Fine GICP refines x/y/yaw and validates overlap before publishing.
//
// Convention: FAST-LIO reloc input is T_robot^odom. After a successful
// relocalization this system keeps map->odom as identity, so the map pose is
// published with frame_id=odom on /reloc/cloud_align.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

namespace
{
struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const std::size_t hx = std::hash<int>{}(key.x);
    const std::size_t hy = std::hash<int>{}(key.y);
    const std::size_t hz = std::hash<int>{}(key.z);
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

VoxelKey voxel_key(const pcl::PointXYZ & point, double voxel)
{
  return {
    static_cast<int>(std::floor(point.x / voxel)),
    static_cast<int>(std::floor(point.y / voxel)),
    static_cast<int>(std::floor(point.z / voxel))};
}

double yaw_deg(const Eigen::Matrix4f & transform)
{
  return std::atan2(transform(1, 0), transform(0, 0)) * 180.0 / M_PI;
}

double wrap_deg(double angle)
{
  while (angle > 180.0) {
    angle -= 360.0;
  }
  while (angle < -180.0) {
    angle += 360.0;
  }
  return angle;
}
}  // namespace

class IcpRelocalizer : public rclcpp::Node
{
public:
  IcpRelocalizer() : Node("icp_relocalizer")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    const double legacy_map_voxel = declare_parameter<double>("map_voxel", 0.2);
    const double legacy_source_voxel = declare_parameter<double>("source_voxel", 0.2);
    const double legacy_corr_dist =
      declare_parameter<double>("max_correspondence_distance", 1.0);
    const int legacy_iterations = declare_parameter<int>("max_iterations", 50);
    const double legacy_fitness = declare_parameter<double>("fitness_threshold", 0.3);

    coarse_map_voxel_ =
      declare_parameter<double>("coarse_map_voxel", std::max(legacy_map_voxel, 0.3));
    coarse_source_voxel_ =
      declare_parameter<double>("coarse_source_voxel", std::max(legacy_source_voxel, 0.3));
    fine_map_voxel_ =
      declare_parameter<double>("fine_map_voxel", std::min(legacy_map_voxel, 0.1));
    fine_source_voxel_ =
      declare_parameter<double>("fine_source_voxel", std::min(legacy_source_voxel, 0.1));
    z_min_ = declare_parameter<double>("z_min", -0.5);
    z_max_ = declare_parameter<double>("z_max", 1.5);
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    request_topic_ = declare_parameter<std::string>("request_topic", "/reloc/request");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/reloc/cloud_align");

    coarse_max_correspondence_distance_ = declare_parameter<double>(
      "coarse_max_correspondence_distance", std::max(legacy_corr_dist, 1.5));
    fine_max_correspondence_distance_ = declare_parameter<double>(
      "fine_max_correspondence_distance", std::min(legacy_corr_dist, 0.5));
    coarse_max_iterations_ =
      declare_parameter<int>("coarse_max_iterations", legacy_iterations);
    fine_max_iterations_ =
      declare_parameter<int>("fine_max_iterations", std::max(legacy_iterations, 80));
    coarse_fitness_threshold_ =
      declare_parameter<double>("coarse_fitness_threshold", std::max(legacy_fitness, 0.7));
    fine_fitness_threshold_ =
      declare_parameter<double>("fine_fitness_threshold", std::min(legacy_fitness, 0.25));
    gicp_correspondence_randomness_ =
      declare_parameter<int>("gicp_correspondence_randomness", 20);

    min_source_points_ = declare_parameter<int>("min_source_points", 200);
    min_map_consistent_points_ =
      declare_parameter<int>("min_map_consistent_points", 200);
    min_correspondences_ = declare_parameter<int>("min_correspondences", 300);
    min_overlap_ratio_ = declare_parameter<double>("min_overlap_ratio", 0.35);
    overlap_distance_ = declare_parameter<double>(
      "overlap_distance", fine_max_correspondence_distance_);
    map_consistency_distance_ =
      declare_parameter<double>("map_consistency_distance", 0.5);

    startup_delay_ = declare_parameter<double>("startup_delay", 5.0);
    max_roll_pitch_deg_ = declare_parameter<double>("max_roll_pitch_deg", 10.0);
    max_correction_dist_ = declare_parameter<double>("max_correction_dist", 2.0);
    max_yaw_correction_deg_ =
      declare_parameter<double>("max_yaw_correction_deg", 35.0);
    accumulate_frames_ = declare_parameter<int>("accumulate_frames", 30);
    use_odom_as_initial_guess_ =
      declare_parameter<bool>("use_odom_as_initial_guess", true);
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_z_ = declare_parameter<double>("initial_z", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    one_shot_ = declare_parameter<bool>("one_shot", true);
    auto_relocalize_on_start_ =
      declare_parameter<bool>("auto_relocalize_on_start", true);
    cooldown_ = declare_parameter<double>("cooldown", 3.0);
    publish_static_map_to_odom_ =
      declare_parameter<bool>("publish_static_map_to_odom", true);

    time_consistency_filter_en_ =
      declare_parameter<bool>("time_consistency_filter_en", true);
    time_consistency_voxel_ =
      declare_parameter<double>("time_consistency_voxel", 0.15);
    time_consistency_min_ratio_ =
      declare_parameter<double>("time_consistency_min_ratio", 0.35);
    time_consistency_min_frames_ =
      declare_parameter<int>("time_consistency_min_frames", 3);
    debug_publish_clouds_ = declare_parameter<bool>("debug_publish_clouds", true);

    if (pcd_file_.empty()) {
      throw std::runtime_error("参数 pcd_file 未设置，需要指向先验 PCD 地图");
    }
    if (z_max_ <= z_min_) {
      throw std::runtime_error("z_max must be greater than z_min");
    }
    if (fine_map_voxel_ <= 0.0 || fine_source_voxel_ <= 0.0 ||
        coarse_map_voxel_ <= 0.0 || coarse_source_voxel_ <= 0.0) {
      throw std::runtime_error("voxel sizes must be positive");
    }

    load_prior_map();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    if (publish_static_map_to_odom_) {
      publish_map_to_odom();
      RCLCPP_INFO(
        get_logger(), "已发布静态 map -> odom = 单位阵（重定位后 odom 仍与 map 重合）");
    }

    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    if (debug_publish_clouds_) {
      debug_accumulated_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/accumulated_raw", 1);
      debug_time_filtered_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/time_filtered", 1);
      debug_coarse_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/coarse_aligned", 1);
      debug_map_filtered_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/map_consistent", 1);
      debug_fine_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/fine_aligned", 1);
    }
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&IcpRelocalizer::cloud_callback, this, std::placeholders::_1));
    request_subscription_ = create_subscription<std_msgs::msg::Empty>(
      request_topic_, 10,
      std::bind(&IcpRelocalizer::request_callback, this, std::placeholders::_1));
    active_ = auto_relocalize_on_start_;

    RCLCPP_INFO(
      get_logger(),
      "等待 %s，request=%s auto_start=%s accumulate=%d | coarse ICP voxel %.2f/%.2f dist %.2f | fine GICP voxel %.2f/%.2f dist %.2f | 输出 %s",
      input_topic_.c_str(), request_topic_.c_str(),
      auto_relocalize_on_start_ ? "true" : "false", accumulate_frames_,
      coarse_map_voxel_, coarse_source_voxel_, coarse_max_correspondence_distance_,
      fine_map_voxel_, fine_source_voxel_, fine_max_correspondence_distance_,
      output_topic_.c_str());
    if (!auto_relocalize_on_start_) {
      RCLCPP_INFO(
        get_logger(),
        "auto_relocalize_on_start=false，等待外部请求: ros2 topic pub --once %s std_msgs/msg/Empty '{}'",
        request_topic_.c_str());
    }
  }

private:
  struct RegistrationResult
  {
    bool converged{false};
    double rmse{0.0};
    double elapsed{0.0};
    Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  };

  struct OverlapStats
  {
    int correspondences{0};
    double ratio{0.0};
  };

  void request_callback(const std_msgs::msg::Empty::SharedPtr)
  {
    begin_relocalization("external request", true);
  }

  void begin_relocalization(const std::string & reason, bool capture_guess_now)
  {
    reset_accumulator();
    done_ = false;
    active_ = true;
    start_time_ = std::chrono::steady_clock::now() -
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(startup_delay_));
    last_attempt_ = std::chrono::steady_clock::time_point{};
    have_run_initial_guess_ = false;
    run_initial_guess_used_odom_ = false;

    if (capture_guess_now) {
      run_initial_guess_ = initial_guess(&run_initial_guess_used_odom_);
      have_run_initial_guess_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "收到重定位触发: %s，清空累计并重新开始；本轮初值%s odom->%s",
      reason.c_str(),
      have_run_initial_guess_ ?
      (run_initial_guess_used_odom_ ? "已取自 FAST-LIO odom TF" : "使用 YAML fallback") :
      "将在首帧取 FAST-LIO odom TF",
      base_frame_.c_str());
    if (have_run_initial_guess_) {
      RCLCPP_INFO(
        get_logger(), "请求初值: xyz=(%.3f, %.3f, %.3f) yaw=%.1f°",
        run_initial_guess_(0, 3), run_initial_guess_(1, 3), run_initial_guess_(2, 3),
        yaw_deg(run_initial_guess_));
    }
  }

  PointCloud::Ptr filter_z(const PointCloud::Ptr & input) const
  {
    PointCloud::Ptr z_filtered(new PointCloud());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*z_filtered);
    return z_filtered;
  }

  PointCloud::Ptr downsample(const PointCloud::Ptr & input, double voxel) const
  {
    PointCloud::Ptr downsampled(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(input);
    grid.setLeafSize(
      static_cast<float>(voxel), static_cast<float>(voxel), static_cast<float>(voxel));
    grid.filter(*downsampled);
    return downsampled;
  }

  PointCloud::Ptr filter_and_downsample(const PointCloud::Ptr & input, double voxel) const
  {
    return downsample(filter_z(input), voxel);
  }

  void load_prior_map()
  {
    RCLCPP_INFO(get_logger(), "读取先验地图 %s ...", pcd_file_.c_str());
    PointCloud::Ptr raw(new PointCloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *raw) != 0) {
      throw std::runtime_error("读不了 PCD 文件: " + pcd_file_);
    }

    target_filtered_ = filter_z(raw);
    target_coarse_ = downsample(target_filtered_, coarse_map_voxel_);
    target_fine_ = downsample(target_filtered_, fine_map_voxel_);
    if (target_coarse_->size() < 50 || target_fine_->size() < 50) {
      throw std::runtime_error(
        "先验地图在 z∈[" + std::to_string(z_min_) + ", " +
        std::to_string(z_max_) + "] 内点数不足，检查 pcd_file 和高度带");
    }
    target_fine_tree_.setInputCloud(target_fine_);
    RCLCPP_INFO(
      get_logger(),
      "先验地图 raw=%zu z_filtered=%zu coarse=%zu(%.2fm) fine=%zu(%.2fm)",
      raw->size(), target_filtered_->size(), target_coarse_->size(), coarse_map_voxel_,
      target_fine_->size(), fine_map_voxel_);
  }

  void publish_map_to_odom()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = odom_frame_;
    tf.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(tf);
  }

  void reset_accumulator()
  {
    accumulated_raw_->clear();
    frame_clouds_.clear();
    frame_count_ = 0;
  }

  bool lookup(
    const std::string & target_frame, const std::string & source_frame,
    Eigen::Matrix4f & out)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      Eigen::Quaternionf q(
        static_cast<float>(tf.transform.rotation.w),
        static_cast<float>(tf.transform.rotation.x),
        static_cast<float>(tf.transform.rotation.y),
        static_cast<float>(tf.transform.rotation.z));
      out = Eigen::Matrix4f::Identity();
      out.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
      out.block<3, 1>(0, 3) = Eigen::Vector3f(
        static_cast<float>(tf.transform.translation.x),
        static_cast<float>(tf.transform.translation.y),
        static_cast<float>(tf.transform.translation.z));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "拿不到 %s <- %s 的 TF: %s",
        target_frame.c_str(), source_frame.c_str(), e.what());
      return false;
    }
  }

  Eigen::Matrix4f initial_guess(bool * used_odom = nullptr)
  {
    if (used_odom) {
      *used_odom = false;
    }
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(static_cast<float>(initial_yaw_), Eigen::Vector3f::UnitZ())
      .toRotationMatrix();
    guess.block<3, 1>(0, 3) = Eigen::Vector3f(
      static_cast<float>(initial_x_), static_cast<float>(initial_y_),
      static_cast<float>(initial_z_));

    if (!use_odom_as_initial_guess_) {
      return guess;
    }
    Eigen::Matrix4f odom;
    if (!lookup(odom_frame_, base_frame_, odom)) {
      return guess;
    }
    if (used_odom) {
      *used_odom = true;
    }
    return odom * guess;
  }

  PointCloud::Ptr time_consistency_filter() const
  {
    PointCloud::Ptr output(new PointCloud());
    if (!time_consistency_filter_en_ || frame_clouds_.size() < 2 ||
        time_consistency_voxel_ <= 0.0) {
      *output = *accumulated_raw_;
      return output;
    }

    std::unordered_map<VoxelKey, int, VoxelKeyHash> frame_counts;
    for (const auto & frame : frame_clouds_) {
      std::unordered_set<VoxelKey, VoxelKeyHash> seen;
      for (const auto & point : frame->points) {
        seen.insert(voxel_key(point, time_consistency_voxel_));
      }
      for (const auto & key : seen) {
        ++frame_counts[key];
      }
    }

    const int ratio_min = static_cast<int>(
      std::ceil(static_cast<double>(frame_clouds_.size()) * time_consistency_min_ratio_));
    const int min_frames = std::clamp(
      std::max(time_consistency_min_frames_, ratio_min), 1,
      static_cast<int>(frame_clouds_.size()));
    output->reserve(accumulated_raw_->size());
    for (const auto & frame : frame_clouds_) {
      for (const auto & point : frame->points) {
        const auto it = frame_counts.find(voxel_key(point, time_consistency_voxel_));
        if (it != frame_counts.end() && it->second >= min_frames) {
          output->push_back(point);
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "时间一致性过滤: %zu -> %zu 点 | voxel=%.2fm min_frames=%d/%zu",
      accumulated_raw_->size(), output->size(), time_consistency_voxel_, min_frames,
      frame_clouds_.size());
    return output;
  }

  RegistrationResult run_coarse_icp(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & guess) const
  {
    const auto started = std::chrono::steady_clock::now();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target_coarse_);
    icp.setMaxCorrespondenceDistance(coarse_max_correspondence_distance_);
    icp.setMaximumIterations(coarse_max_iterations_);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-4);

    PointCloud aligned;
    icp.align(aligned, guess);

    RegistrationResult result;
    result.converged = icp.hasConverged();
    result.transform = icp.getFinalTransformation();
    result.rmse = std::sqrt(icp.getFitnessScore(coarse_max_correspondence_distance_));
    result.elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return result;
  }

  RegistrationResult run_fine_gicp(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & guess) const
  {
    const auto started = std::chrono::steady_clock::now();
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target_fine_);
    gicp.setMaxCorrespondenceDistance(fine_max_correspondence_distance_);
    gicp.setMaximumIterations(fine_max_iterations_);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setEuclideanFitnessEpsilon(1e-5);
    gicp.setCorrespondenceRandomness(gicp_correspondence_randomness_);

    PointCloud aligned;
    gicp.align(aligned, guess);

    RegistrationResult result;
    result.converged = gicp.hasConverged();
    result.transform = gicp.getFinalTransformation();
    result.rmse = std::sqrt(gicp.getFitnessScore(fine_max_correspondence_distance_));
    result.elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return result;
  }

  PointCloud::Ptr filter_by_map_consistency(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & coarse_pose) const
  {
    PointCloud::Ptr filtered(new PointCloud());
    filtered->reserve(source->size());

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    const float max_dist2 =
      static_cast<float>(map_consistency_distance_ * map_consistency_distance_);
    for (const auto & point : source->points) {
      const Eigen::Vector4f p_base(point.x, point.y, point.z, 1.0f);
      const Eigen::Vector4f p_map = coarse_pose * p_base;
      pcl::PointXYZ query(p_map.x(), p_map.y(), p_map.z());
      if (target_fine_tree_.nearestKSearch(query, 1, indices, distances) > 0 &&
          distances[0] <= max_dist2) {
        filtered->push_back(point);
      }
    }

    RCLCPP_INFO(
      get_logger(), "地图一致性过滤: %zu -> %zu 点 | distance<=%.2fm",
      source->size(), filtered->size(), map_consistency_distance_);
    return filtered;
  }

  OverlapStats compute_overlap(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & transform,
    double distance) const
  {
    OverlapStats stats;
    if (source->empty()) {
      return stats;
    }

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    const float max_dist2 = static_cast<float>(distance * distance);
    for (const auto & point : source->points) {
      const Eigen::Vector4f p_base(point.x, point.y, point.z, 1.0f);
      const Eigen::Vector4f p_map = transform * p_base;
      pcl::PointXYZ query(p_map.x(), p_map.y(), p_map.z());
      if (target_fine_tree_.nearestKSearch(query, 1, indices, distances) > 0 &&
          distances[0] <= max_dist2) {
        ++stats.correspondences;
      }
    }
    stats.ratio = static_cast<double>(stats.correspondences) /
      static_cast<double>(source->size());
    return stats;
  }

  bool validate_final_result(
    const RegistrationResult & fine, const Eigen::Matrix4f & guess,
    const OverlapStats & overlap) const
  {
    if (!fine.converged) {
      RCLCPP_WARN(get_logger(), "REJECT: Fine GICP 未收敛");
      return false;
    }
    if (fine.rmse > fine_fitness_threshold_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: Fine GICP 残差 %.3fm > %.3fm",
        fine.rmse, fine_fitness_threshold_);
      return false;
    }
    if (overlap.correspondences < min_correspondences_ ||
        overlap.ratio < min_overlap_ratio_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: overlap %.1f%% (%d pts) 不足，阈值 %.1f%% / %d pts",
        overlap.ratio * 100.0, overlap.correspondences,
        min_overlap_ratio_ * 100.0, min_correspondences_);
      return false;
    }

    const Eigen::Matrix3f rotation = fine.transform.block<3, 3>(0, 0);
    const double pitch = std::asin(
      std::clamp(-static_cast<double>(rotation(2, 0)), -1.0, 1.0));
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double tilt_deg = std::max(std::abs(roll), std::abs(pitch)) * 180.0 / M_PI;
    if (tilt_deg > max_roll_pitch_deg_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: roll/pitch=%.1f° > %.1f°",
        tilt_deg, max_roll_pitch_deg_);
      return false;
    }

    const Eigen::Matrix4f correction = fine.transform * guess.inverse();
    const double correction_dist = correction.block<3, 1>(0, 3).norm();
    const double yaw_correction = std::abs(wrap_deg(yaw_deg(fine.transform) - yaw_deg(guess)));
    if (correction_dist > max_correction_dist_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: 相对初值平移 %.2fm > %.2fm",
        correction_dist, max_correction_dist_);
      return false;
    }
    if (yaw_correction > max_yaw_correction_deg_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: yaw 修正 %.1f° > %.1f°",
        yaw_correction, max_yaw_correction_deg_);
      return false;
    }
    return true;
  }

  void publish_debug_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloud::Ptr & cloud, const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    if (!debug_publish_clouds_ || !publisher || !cloud) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    publisher->publish(msg);
  }

  void publish_aligned_debug_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloud::Ptr & cloud, const Eigen::Matrix4f & transform,
    const rclcpp::Time & stamp) const
  {
    if (!debug_publish_clouds_ || !publisher || !cloud) {
      return;
    }
    PointCloud::Ptr aligned(new PointCloud());
    pcl::transformPointCloud(*cloud, *aligned, transform);
    publish_debug_cloud(publisher, aligned, odom_frame_, stamp);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!active_ || done_) {
      return;
    }
    const auto now_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_time - start_time_).count() < startup_delay_) {
      return;
    }
    if (std::chrono::duration<double>(now_time - last_attempt_).count() < cooldown_) {
      return;
    }

    PointCloud::Ptr cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
      Eigen::Matrix4f transform;
      if (!lookup(base_frame_, msg->header.frame_id, transform)) {
        return;
      }
      pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    PointCloud::Ptr frame = filter_and_downsample(cloud, fine_source_voxel_);
    if (static_cast<int>(frame->size()) < min_source_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "实时点云只剩 %zu 个点，跳过（阈值 %d）",
        frame->size(), min_source_points_);
      return;
    }

    if (!have_run_initial_guess_) {
      run_initial_guess_ = initial_guess(&run_initial_guess_used_odom_);
      have_run_initial_guess_ = true;
      RCLCPP_INFO(
        get_logger(), "本轮自动初值[%s]: xyz=(%.3f, %.3f, %.3f) yaw=%.1f°",
        run_initial_guess_used_odom_ ? "FAST-LIO odom TF" : "YAML fallback",
        run_initial_guess_(0, 3), run_initial_guess_(1, 3), run_initial_guess_(2, 3),
        yaw_deg(run_initial_guess_));
    }

    *accumulated_raw_ += *frame;
    frame_clouds_.push_back(frame);
    ++frame_count_;
    if (frame_count_ < accumulate_frames_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "累积点云 %d/%d 帧（%zu 点，稳住别动）",
        frame_count_, accumulate_frames_, accumulated_raw_->size());
      return;
    }

    last_attempt_ = now_time;
    const auto pipeline_started = std::chrono::steady_clock::now();
    const auto stamp = msg->header.stamp;
    publish_debug_cloud(debug_accumulated_pub_, accumulated_raw_, base_frame_, stamp);

    PointCloud::Ptr stable_cloud = time_consistency_filter();
    publish_debug_cloud(debug_time_filtered_pub_, stable_cloud, base_frame_, stamp);
    if (static_cast<int>(stable_cloud->size()) < min_source_points_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: 时间过滤后点数 %zu < %d，重置累积",
        stable_cloud->size(), min_source_points_);
      reset_accumulator();
      return;
    }

    PointCloud::Ptr coarse_source = downsample(stable_cloud, coarse_source_voxel_);
    PointCloud::Ptr fine_source = downsample(stable_cloud, fine_source_voxel_);
    if (static_cast<int>(coarse_source->size()) < min_source_points_ ||
        static_cast<int>(fine_source->size()) < min_source_points_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: 降采样后点数不足 coarse=%zu fine=%zu 阈值=%d",
        coarse_source->size(), fine_source->size(), min_source_points_);
      reset_accumulator();
      return;
    }

    const Eigen::Matrix4f guess = run_initial_guess_;
    RCLCPP_INFO(
      get_logger(), "初始 pose: xyz=(%.3f, %.3f, %.3f) yaw=%.1f°",
      guess(0, 3), guess(1, 3), guess(2, 3), yaw_deg(guess));

    const RegistrationResult coarse = run_coarse_icp(coarse_source, guess);
    RCLCPP_INFO(
      get_logger(),
      "粗配准: converged=%s xyz=(%.3f, %.3f, %.3f) yaw=%.1f° yaw_delta=%.1f° fitness=%.3fm time=%.2fs",
      coarse.converged ? "true" : "false", coarse.transform(0, 3), coarse.transform(1, 3),
      coarse.transform(2, 3), yaw_deg(coarse.transform),
      wrap_deg(yaw_deg(coarse.transform) - yaw_deg(guess)), coarse.rmse, coarse.elapsed);
    publish_aligned_debug_cloud(debug_coarse_pub_, coarse_source, coarse.transform, stamp);

    if (!coarse.converged) {
      RCLCPP_WARN(get_logger(), "REJECT: Coarse ICP 未收敛");
      reset_accumulator();
      return;
    }
    if (coarse.rmse > coarse_fitness_threshold_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: Coarse ICP 残差 %.3fm > %.3fm",
        coarse.rmse, coarse_fitness_threshold_);
      reset_accumulator();
      return;
    }

    PointCloud::Ptr map_consistent =
      filter_by_map_consistency(fine_source, coarse.transform);
    publish_aligned_debug_cloud(
      debug_map_filtered_pub_, map_consistent, coarse.transform, stamp);
    if (static_cast<int>(map_consistent->size()) < min_map_consistent_points_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: 地图一致点 %zu < %d",
        map_consistent->size(), min_map_consistent_points_);
      reset_accumulator();
      return;
    }

    const RegistrationResult fine = run_fine_gicp(map_consistent, coarse.transform);
    const OverlapStats overlap =
      compute_overlap(map_consistent, fine.transform, overlap_distance_);
    RCLCPP_INFO(
      get_logger(),
      "精配准: converged=%s xyz=(%.3f, %.3f, %.3f) yaw=%.1f° yaw_delta_init=%.1f° yaw_delta_coarse=%.1f° fitness=%.3fm overlap=%.1f%%(%d/%zu) time=%.2fs",
      fine.converged ? "true" : "false", fine.transform(0, 3), fine.transform(1, 3),
      fine.transform(2, 3), yaw_deg(fine.transform),
      wrap_deg(yaw_deg(fine.transform) - yaw_deg(guess)),
      wrap_deg(yaw_deg(fine.transform) - yaw_deg(coarse.transform)), fine.rmse,
      overlap.ratio * 100.0, overlap.correspondences, map_consistent->size(),
      fine.elapsed);
    publish_aligned_debug_cloud(debug_fine_pub_, map_consistent, fine.transform, stamp);

    if (!validate_final_result(fine, guess, overlap)) {
      reset_accumulator();
      return;
    }

    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - pipeline_started).count();
    RCLCPP_INFO(
      get_logger(),
      "ACCEPT: coarse_fitness=%.3fm fine_fitness=%.3fm overlap=%.1f%% corr=%d total_time=%.2fs",
      coarse.rmse, fine.rmse, overlap.ratio * 100.0, overlap.correspondences, elapsed);
    publish_pose(msg, fine.transform, fine.rmse, overlap, elapsed);
  }

  void publish_pose(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const Eigen::Matrix4f & transform, double rmse, const OverlapStats & overlap,
    double elapsed)
  {
    const Eigen::Vector3f t = transform.block<3, 1>(0, 3);
    Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
    q.normalize();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = odom_frame_;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = t.x();
    pose.pose.position.y = t.y();
    pose.pose.position.z = t.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    publisher_->publish(pose);

    RCLCPP_INFO(
      get_logger(),
      "重定位成功：xyz=(%.3f, %.3f, %.3f) yaw=%.1f° | Fine 残差 %.3fm | overlap %.1f%% | 用时 %.2fs | 已发到 %s",
      t.x(), t.y(), t.z(), yaw_deg(transform), rmse, overlap.ratio * 100.0, elapsed,
      output_topic_.c_str());

    if (publish_static_map_to_odom_) {
      publish_map_to_odom();
    }

    if (one_shot_) {
      active_ = false;
      done_ = true;
      RCLCPP_INFO(get_logger(), "one_shot=true，停止后续匹配");
    } else {
      reset_accumulator();
    }
  }

  std::string pcd_file_;
  double coarse_map_voxel_{0.3};
  double coarse_source_voxel_{0.3};
  double fine_map_voxel_{0.1};
  double fine_source_voxel_{0.1};
  double z_min_{-0.5};
  double z_max_{1.5};
  std::string input_topic_;
  std::string request_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string output_topic_;
  double coarse_max_correspondence_distance_{1.5};
  double fine_max_correspondence_distance_{0.5};
  int coarse_max_iterations_{50};
  int fine_max_iterations_{80};
  double coarse_fitness_threshold_{0.7};
  double fine_fitness_threshold_{0.25};
  int gicp_correspondence_randomness_{20};
  int min_source_points_{200};
  int min_map_consistent_points_{200};
  int min_correspondences_{300};
  double min_overlap_ratio_{0.35};
  double overlap_distance_{0.5};
  double map_consistency_distance_{0.5};
  int accumulate_frames_{30};
  PointCloud::Ptr accumulated_raw_{new PointCloud()};
  std::vector<PointCloud::Ptr> frame_clouds_;
  int frame_count_{0};
  double startup_delay_{5.0};
  double max_roll_pitch_deg_{10.0};
  double max_correction_dist_{2.0};
  double max_yaw_correction_deg_{35.0};
  bool use_odom_as_initial_guess_{true};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_z_{0.0};
  double initial_yaw_{0.0};
  bool one_shot_{true};
  bool auto_relocalize_on_start_{true};
  double cooldown_{3.0};
  bool publish_static_map_to_odom_{true};
  bool time_consistency_filter_en_{true};
  double time_consistency_voxel_{0.15};
  double time_consistency_min_ratio_{0.35};
  int time_consistency_min_frames_{3};
  bool debug_publish_clouds_{true};

  PointCloud::Ptr target_filtered_;
  PointCloud::Ptr target_coarse_;
  PointCloud::Ptr target_fine_;
  pcl::KdTreeFLANN<pcl::PointXYZ> target_fine_tree_;
  std::chrono::steady_clock::time_point last_attempt_{};
  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
  Eigen::Matrix4f run_initial_guess_{Eigen::Matrix4f::Identity()};
  bool have_run_initial_guess_{false};
  bool run_initial_guess_used_odom_{false};
  bool active_{true};
  bool done_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_accumulated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_time_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_coarse_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_map_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_fine_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr request_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<IcpRelocalizer>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("icp_relocalizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
