// 自车几何滤波：用 YAML 内置基本体（或可选的 URDF 几何）过滤点云。
//
// 思路：URDF 里所有 link 都由固定关节连到 base_link，所以能把每个 link 的
// collision（没有则退回 visual）几何体，连同它相对 base_link 的位姿，化成
// base_link 系下的一组基本体（box / cylinder / sphere）。雷达点先用
// T_lidar^base 转到车体系，再逐个基本体测试：落在任意一个内部就丢弃。
//
// 相比固定半径的球面盲区或角度扇形，这样得到的是"车体真实形状"的盲区，
// 车体改了（换底盘、加安装件）只要改 URDF，代码不用动。
#ifndef FAST_LIO_SELF_FILTER_H
#define FAST_LIO_SELF_FILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <map>
#include <memory>
#include <utility>
#include <string>
#include <vector>

namespace fast_lio
{

class RobotSelfFilter
{
public:
  struct Primitive
  {
    enum class Type { kBox, kCylinder, kSphere };

    Type type = Type::kBox;
    // 基本体中心在 base_link 系下的位置。
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    // 基本体自身坐标轴 -> base_link 的旋转（列为基本体的 x/y/z 轴）。
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    // box 的半长；已经含 margin。
    Eigen::Vector3d half_extent = Eigen::Vector3d::Zero();
    // cylinder / sphere 的半径；已经含 margin。
    double radius = 0.0;
    // cylinder 沿自身 +z 的长度；已经含 margin。
    double length = 0.0;
    // 来自哪个 link，便于排查。
    std::string link;
  };

  // 从固定 box 构建自车过滤体。尺寸是最终过滤尺寸（已包含所需安全边界），
  // 因而不依赖 /robot_description、URDF 或 robot_state_publisher。
  bool initFromBox(
    const Eigen::Vector3d & center, const Eigen::Vector3d & size,
    std::string * report)
  {
    if ((size.array() <= 0.0).any()) {
      if (report) {
        *report = "box_size 的三个分量必须都大于 0";
      }
      return false;
    }
    Primitive box;
    box.type = Primitive::Type::kBox;
    box.center = center;
    box.half_extent = size * 0.5;
    box.link = "self_filter.box";
    primitives_.assign(1, box);
    ready_.store(true, std::memory_order_release);
    if (report) {
      *report = "自车几何滤波：内置 box center=(" +
        std::to_string(center.x()) + ", " + std::to_string(center.y()) + ", " +
        std::to_string(center.z()) + ") size=(" + std::to_string(size.x()) + ", " +
        std::to_string(size.y()) + ", " + std::to_string(size.z()) + ")";
    }
    return true;
  }

  // 从 URDF 字符串构建几何。margin 为几何外扩量（m），用于吃掉建图噪声和
  // 车体表面本身的回波。include_visual 为 true 时，没有 collision 的 link
  // 退回用 visual 几何（本仓库的 mid360 mesh 就是只有 visual）。
  // report 非空时写入一份人类可读的几何清单。
  // 返回是否成功；成功之前 ready() 一直是 false，此时上层不过滤。
  bool initFromUrdf(
    const std::string & urdf_xml, const std::string & base_link, double margin,
    bool include_visual, std::string * report)
  {
    std::vector<Primitive> primitives;
    std::string log;

    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      return false;
    }
    if (model.links_.find(base_link) == model.links_.end()) {
      if (report) {
        *report = "URDF 里没有名为 '" + base_link + "' 的 link";
      }
      return false;
    }

    // 1) 收集固定关节：child -> (parent, T_child^parent)。
    //    urdfdom 的 parent_to_joint_origin_transform 就是关节系（= 子 link 系）
    //    在父 link 系下的位姿。
    std::map<std::string, std::pair<std::string, Eigen::Isometry3d>> parents;
    for (const auto & entry : model.joints_) {
      const urdf::JointSharedPtr & joint = entry.second;
      if (joint->type != urdf::Joint::FIXED) {
        log += "  跳过非固定关节 " + joint->name + "（自车滤波只处理固定几何）\n";
        continue;
      }
      parents[joint->child_link_name] =
        {joint->parent_link_name, poseToIsometry(joint->parent_to_joint_origin_transform)};
    }

    // 2) 逐 link 把几何搬到 base_link 系。
    for (const auto & entry : model.links_) {
      const std::string & link_name = entry.first;
      const urdf::LinkSharedPtr & link = entry.second;

      Eigen::Isometry3d T_link_in_base;
      if (!poseInBase(link_name, base_link, parents, &T_link_in_base)) {
        continue;  // 不在 base_link 这条链上（例如挂在别的根上），忽略
      }

      const bool use_collision = !link->collision_array.empty();
      if (!use_collision && !include_visual) {
        continue;
      }

      // collision_array 与 visual_array 是两种类型，用泛型 lambda 各走一遍。
      const auto collect = [&](const auto & shapes, const char * source) {
        for (const auto & shape : shapes) {
          if (!shape || !shape->geometry) {
            continue;
          }
          Primitive prim;
          prim.link = link_name;

          const Eigen::Isometry3d T_geom_in_link = poseToIsometry(shape->origin);
          const Eigen::Isometry3d T_geom_in_base = T_link_in_base * T_geom_in_link;
          prim.center = T_geom_in_base.translation();
          prim.rotation = T_geom_in_base.rotation();

          switch (shape->geometry->type) {
            case urdf::Geometry::BOX: {
              const auto box = std::static_pointer_cast<urdf::Box>(shape->geometry);
              prim.type = Primitive::Type::kBox;
              // 外扩 margin：把半长直接放大，等价于把盒子按 margin 膨胀。
              prim.half_extent =
                Eigen::Vector3d(box->dim.x * 0.5 + margin, box->dim.y * 0.5 + margin,
                                box->dim.z * 0.5 + margin);
              log += logLine(prim, std::string(source) + " box");
              break;
            }
            case urdf::Geometry::CYLINDER: {
              const auto cyl = std::static_pointer_cast<urdf::Cylinder>(shape->geometry);
              prim.type = Primitive::Type::kCylinder;
              prim.radius = cyl->radius + margin;
              prim.length = cyl->length + 2.0 * margin;
              log += logLine(prim, std::string(source) + " cylinder");
              break;
            }
            case urdf::Geometry::SPHERE: {
              const auto sph = std::static_pointer_cast<urdf::Sphere>(shape->geometry);
              prim.type = Primitive::Type::kSphere;
              prim.radius = sph->radius + margin;
              log += logLine(prim, std::string(source) + " sphere");
              break;
            }
            default:
              // mesh 需要读 STL 才能得到形状，这里不猜；AABB 逼近会把盒子撑大，
              // 反而误删真实环境点，所以直接跳过并提示。
              log += "  跳过 " + link_name + " 的 mesh 几何（未解析 mesh 外形）\n";
              continue;
          }
          primitives.push_back(prim);
        }
      };

      if (use_collision) {
        collect(link->collision_array, "collision");
      } else {
        collect(link->visual_array, "visual");
      }
    }

    if (primitives.empty()) {
      if (report) {
        *report = "URDF 里没有可用的几何（全是 mesh 或不在 base_link 链上）";
      }
      return false;
    }

    primitives_.swap(primitives);
    ready_.store(true, std::memory_order_release);
    if (report) {
      *report = "自车几何滤波：base_link=" + base_link + "，margin=" +
        std::to_string(margin) + "m，共 " + std::to_string(primitives_.size()) +
        " 个基本体\n" + log;
    }
    return true;
  }

  bool ready() const { return ready_.load(std::memory_order_acquire); }
  std::size_t size() const { return primitives_.size(); }
  const std::vector<Primitive> & primitives() const { return primitives_; }

  // p 在 base_link 系。落在任一基本体（含 margin）内部 -> true，应当丢弃。
  // 热路径：保持内联、无分配。
  bool inside(const Eigen::Vector3d & p) const
  {
    for (const Primitive & prim : primitives_) {
      const Eigen::Vector3d q = prim.rotation.transpose() * (p - prim.center);
      switch (prim.type) {
        case Primitive::Type::kBox:
          if (std::abs(q.x()) <= prim.half_extent.x() &&
            std::abs(q.y()) <= prim.half_extent.y() &&
            std::abs(q.z()) <= prim.half_extent.z())
          {
            return true;
          }
          break;
        case Primitive::Type::kCylinder:
          if (q.x() * q.x() + q.y() * q.y() <= prim.radius * prim.radius &&
            std::abs(q.z()) <= 0.5 * prim.length)
          {
            return true;
          }
          break;
        case Primitive::Type::kSphere:
          if (q.squaredNorm() <= prim.radius * prim.radius) {
            return true;
          }
          break;
      }
    }
    return false;
  }

private:
  static Eigen::Isometry3d poseToIsometry(const urdf::Pose & pose)
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    q.normalize();
    T.linear() = q.toRotationMatrix();
    return T;
  }

  // 沿着固定关节链把 T_link^base 累乘出来（全部是固定关节，无需 FK 求解器）。
  static bool poseInBase(
    const std::string & link, const std::string & base_link,
    const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>> & parents,
    Eigen::Isometry3d * T_link_in_base)
  {
    if (link == base_link) {
      *T_link_in_base = Eigen::Isometry3d::Identity();
      return true;
    }
    // 自下而上收集 T_child^parent，再自顶向下乘起来。
    std::vector<Eigen::Isometry3d> chain;
    std::string current = link;
    for (int guard = 0; guard < 256; ++guard) {
      const auto it = parents.find(current);
      if (it == parents.end()) {
        return false;
      }
      chain.push_back(it->second.second);
      current = it->second.first;
      if (current == base_link) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (auto rit = chain.rbegin(); rit != chain.rend(); ++rit) {
          T = T * (*rit);
        }
        *T_link_in_base = T;
        return true;
      }
    }
    return false;
  }

  // 把尺寸也打出来，方便直接和 measurement_params_real.yaml 对账。
  static std::string logLine(const Primitive & prim, const std::string & what)
  {
    char buffer[320] = {0};
    const char * type = "box";
    if (prim.type == Primitive::Type::kCylinder) {
      type = "cylinder";
    } else if (prim.type == Primitive::Type::kSphere) {
      type = "sphere";
    }
    switch (prim.type) {
      case Primitive::Type::kBox:
        std::snprintf(
          buffer, sizeof(buffer),
          "  %-20s %-9s %-8s center=(%7.3f, %7.3f, %7.3f)  size=(%.3f, %.3f, %.3f)\n",
          prim.link.c_str(), type, what.c_str(), prim.center.x(), prim.center.y(),
          prim.center.z(), prim.half_extent.x() * 2.0, prim.half_extent.y() * 2.0,
          prim.half_extent.z() * 2.0);
        break;
      case Primitive::Type::kCylinder:
        std::snprintf(
          buffer, sizeof(buffer),
          "  %-20s %-9s %-8s center=(%7.3f, %7.3f, %7.3f)  radius=%.3f length=%.3f\n",
          prim.link.c_str(), type, what.c_str(), prim.center.x(), prim.center.y(),
          prim.center.z(), prim.radius, prim.length);
        break;
      case Primitive::Type::kSphere:
        std::snprintf(
          buffer, sizeof(buffer),
          "  %-20s %-9s %-8s center=(%7.3f, %7.3f, %7.3f)  radius=%.3f\n",
          prim.link.c_str(), type, what.c_str(), prim.center.x(), prim.center.y(),
          prim.center.z(), prim.radius);
        break;
    }
    return std::string(buffer);
  }

  std::vector<Primitive> primitives_;
  std::atomic<bool> ready_{false};
};

}  // namespace fast_lio

#endif  // FAST_LIO_SELF_FILTER_H
