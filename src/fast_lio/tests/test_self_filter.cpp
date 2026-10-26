#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <stdexcept>
#include <type_traits>

#include "preprocess.h"

namespace
{
int failures = 0;
int checks = 0;
const Eigen::Matrix3d rotation =
  (Eigen::Matrix3d() << 0, -1, 0, -1, 0, 0, 0, 0, -1).finished();
const Eigen::Vector3d translation(0.1515, 0.0, 0.42025);

void check(bool ok, const std::string &name)
{
  ++checks;
  failures += !ok;
  std::cout << (ok ? "PASS " : "FAIL ") << name << '\n';
}

void configure(Preprocess &p, bool enabled = true)
{
  p.N_SCANS = 4;
  p.point_filter_num = 1;
  p.R_lidar_to_robot = rotation;
  p.t_lidar_to_robot = translation;
  if (enabled) {
    auto filter = std::make_shared<fast_lio::RobotSelfFilter>();
    std::string report;
    if (!filter->initFromBox(Eigen::Vector3d(0, 0, 0.7), Eigen::Vector3d(0.8, 0.8, 1.5), &report)) {
      throw std::runtime_error(report);
    }
    p.self_filter = filter;
  }
}

LivoxCustomMsg::SharedPtr livox(const std::vector<Eigen::Vector3d> &points)
{
  auto msg = std::make_shared<LivoxCustomMsg>();
  // The original Livox parser skips point zero; give it a header point.
  msg->points.resize(points.size() + 1);
  msg->point_num = msg->points.size();
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto &pt = msg->points[i + 1];
    pt.x = points[i].x(); pt.y = points[i].y(); pt.z = points[i].z();
    pt.tag = 0x10; pt.line = 0;
    pt.offset_time = static_cast<uint32_t>((i + 1) * 1000);
  }
  return msg;
}

std::size_t run_livox(const std::vector<Eigen::Vector3d> &points, bool feature, bool enabled = true)
{
  Preprocess p;
  configure(p, enabled);
  p.feature_enabled = feature;
  auto out = std::make_shared<PointCloudXYZI>();
  p.process(livox(points), out);
  for (const auto *cloud : {&p.pl_surf, &p.pl_corn}) {
    for (const auto &point : *cloud) {
      const Eigen::Vector3d base = rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
      if (enabled && (base.array() >= Eigen::Vector3d(-0.4, -0.4, -0.05).array()).all() &&
          (base.array() <= Eigen::Vector3d(0.4, 0.4, 1.45).array()).all()) {
        check(false, "feature output must not contain vehicle points");
      }
    }
  }
  return out->size();
}

template<typename Point>
void standard_cloud(int type, const std::string &name)
{
  // First point is in the body, second is just outside its front but within
  // the former 0.30m sphere. The complete transform is needed to distinguish them.
  const Eigen::Vector3d body = rotation.transpose() * (Eigen::Vector3d(0, 0, 1.0) - translation);
  const Eigen::Vector3d near = rotation.transpose() * (Eigen::Vector3d(0.42, 0, 0.42025) - translation);
  pcl::PointCloud<Point> input;
  for (const auto &v : {body, near}) {
    Point point{};
    point.x = v.x(); point.y = v.y(); point.z = v.z(); point.intensity = 10;
    input.push_back(point);
  }
  auto msg = std::make_shared<Pcl2Msg>();
  pcl::toROSMsg(input, *msg);
  auto out = std::make_shared<PointCloudXYZI>();
  Preprocess p;
  configure(p);
  p.lidar_type = type;
  // Velodyne uses an explicit per-point timestamp to avoid its first-return skip.
  if constexpr (std::is_same_v<Point, velodyne_ros::Point>) {
    input[0].time = 0.01f; input[1].time = 0.02f;
    pcl::toROSMsg(input, *msg);
  }
  p.process(msg, out);
  check(out->size() == 1 && std::abs(out->front().y - near.y()) < 1e-5,
        name + " removes body and keeps nearby environmental point");

  // Empty scans and unpopulated rings must not enter unsigned loops.
  input.clear(); pcl::toROSMsg(input, *msg); p.feature_enabled = true;
  p.process(msg, out);
  check(out->empty(), name + " empty feature input");

  for (bool vehicle : {true, false}) {
    input.clear();
    for (int i = 0; i < 80; ++i) {
      Point point{};
      point.x = vehicle ? -0.35f + 0.000625f * i : -0.03f + 0.00075f * i;
      point.y = vehicle ? 0.2f : -0.28f;
      point.z = vehicle ? -0.6f : 0.0f;
      point.intensity = 10;
      if constexpr (std::is_same_v<Point, velodyne_ros::Point>) point.time = 0.01f + 0.001f*i;
      input.push_back(point);
    }
    pcl::toROSMsg(input, *msg); p.process(msg, out);
    check(vehicle ? out->empty() : !out->empty(),
          name + (vehicle ? " feature body returns removed" : " feature nearby plane retained"));
  }
}
}  // namespace

int main()
{
  const Eigen::Vector3d near = rotation.transpose() * (Eigen::Vector3d(0.42, 0, 0.42025) - translation);
  const Eigen::Vector3d body = rotation.transpose() * (Eigen::Vector3d(0, 0, 1.0) - translation);
  check(run_livox({near}, false) == 1, "Livox keeps point outside box within former sphere");
  check(run_livox({body}, false) == 0, "Livox removes body point outside former sphere");
  check(run_livox({Eigen::Vector3d(0.1, 0, 0)}, false, false) == 1,
        "no radius filter when vehicle filter disabled");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  check(run_livox({Eigen::Vector3d(nan, 0, 1), Eigen::Vector3d(inf, 0, 1)}, false) == 0,
        "nonfinite measurements removed");

  std::vector<Eigen::Vector3d> vehicle, near_plane, formerly_crashing, vertical;
  for (int i = 0; i < 80; ++i) {
    vehicle.emplace_back(-0.35 + 0.000625 * i, 0.2, -0.6);
    near_plane.emplace_back(-0.03 + 0.00075 * i, -0.28, 0.0);
    formerly_crashing.emplace_back(-0.2 + 0.005 * i, 0.15, 0.6);
    vertical.emplace_back(0.0, 0.0, 0.6 + 0.005 * i);
  }
  for (bool feature : {false, true}) {
    check(run_livox(vehicle, feature) == 0, "body points removed features=" + std::to_string(feature));
    check(run_livox(near_plane, feature) > 0, "nearby plane retained features=" + std::to_string(feature));
  }
  check(run_livox(formerly_crashing, true) > 0, "formerly crashing all-near-XY scan line");
  check(run_livox(vertical, true) > 0, "zero XY radius is not a blind zone");
  check(run_livox(std::vector<Eigen::Vector3d>(80, Eigen::Vector3d::Zero()), true, false) == 0,
        "all zero direction feature line handled safely");
  for (int count = 0; count <= 10; ++count) {
    auto short_line = formerly_crashing;
    short_line.resize(count);
    run_livox(short_line, true);
  }
  check(true, "short feature lines 0 through 10 handled safely");

  standard_cloud<ouster_ros::Point>(OUST64, "Ouster");
  standard_cloud<velodyne_ros::Point>(VELO16, "Velodyne");
  standard_cloud<robosense_ros::Point>(AIRY, "Airy");
  standard_cloud<unilidar_ros::Point>(UNILIDAR, "Unilidar");
  standard_cloud<pcl::PointXYZI>(MARSIM, "Simulation");
  standard_cloud<kaist_ros::Point>(KAIST, "KAIST");

  fast_lio::RobotSelfFilter filter;
  std::string report;
  check(filter.initFromBox(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), &report), "valid box");
  check(filter.inside(Eigen::Vector3d(0.5, 0, 0)), "box boundary included");
  check(!filter.inside(Eigen::Vector3d(0.5001, 0, 0)), "point beyond box retained");
  for (const Eigen::Vector3d &size : {Eigen::Vector3d(nan, 1, 1), Eigen::Vector3d(inf, 1, 1),
                                    Eigen::Vector3d(0, 1, 1), Eigen::Vector3d(-1, 1, 1)}) {
    check(!filter.initFromBox(Eigen::Vector3d::Zero(), size, &report) && !filter.ready(),
          "invalid size rejected and previous box invalidated");
  }
  check(!filter.initFromBox(Eigen::Vector3d(nan, 0, 0), Eigen::Vector3d::Ones(), &report),
        "nonfinite center rejected");
  Preprocess invalid; configure(invalid); invalid.point_filter_num = 0;
  bool rejected = false;
  try { auto out = std::make_shared<PointCloudXYZI>(); invalid.process(livox({near}), out); }
  catch (const std::runtime_error &) { rejected = true; }
  check(rejected, "invalid sampling parameter rejected before modulo");
  std::cout << checks << " checks, " << failures << " failures\n";
  return failures ? 1 : 0;
}
