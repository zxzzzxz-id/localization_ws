#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <omp.h>

#include "IMU_Processing.hpp"

// Exercise the real initialization and EKF propagation without a ROS graph.
// Reuse ImuProcess across resets, as the relocalization path does.
int main()
{
  ImuProcess imu;
  imu.set_acc_cov(V3D::Constant(0.1));
  imu.set_gyr_cov(V3D::Constant(0.1));
  imu.lidar_type = AVIA;
  const V3D gravity_world(0.0, 0.0, -G_m_s2);
  const V3D gyro_bias(0.001, -0.002, 0.003);
  const std::vector<std::pair<std::string, M3D>> poses{
    {"identity", M3D::Identity()},
    {"yaw", Eigen::AngleAxisd(0.8, V3D::UnitZ()).toRotationMatrix()},
    {"inverted_roll", Eigen::AngleAxisd(M_PI, V3D::UnitX()).toRotationMatrix()},
    {"inverted_pitch", Eigen::AngleAxisd(M_PI, V3D::UnitY()).toRotationMatrix()},
    {"tilted", (Eigen::AngleAxisd(0.7, V3D::UnitZ()) *
                Eigen::AngleAxisd(-0.4, V3D::UnitY()) *
                Eigen::AngleAxisd(0.3, V3D::UnitX())).toRotationMatrix()},
    {"identity_after_reset", M3D::Identity()},
    {"inverted_again", Eigen::AngleAxisd(M_PI, V3D::UnitX()).toRotationMatrix()}
  };
  bool passed = true;
  for (const auto &[name, rotation] : poses) {
    imu.Reset();
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
    state_ikfom initial;
    initial.rot = Eigen::Quaterniond(rotation);
    initial.pos = V3D(1.0, 2.0, 3.0);
    filter.change_x(initial);
    double limits[state_ikfom::DOF];
    std::fill_n(limits, state_ikfom::DOF, 0.001);
    filter.init_dyn_share(get_f, df_dx, df_dw, nullptr, 1, limits);

    // Stationary specific force in the IMU frame, in Livox g units.
    const V3D raw_acc = rotation.transpose() * (-gravity_world) / G_m_s2;
    MeasureGroup measurements;
    measurements.lidar_beg_time = 1.0;
    measurements.lidar_end_time = 1.1;
    // Nine samples exercise gravity initialization without opening the optional
    // IMU log on initialization completion.
    for (int i = 0; i < 9; ++i) {
      auto sample = std::make_shared<ImuMsg>();
      sample->header.stamp.sec = 1;
      sample->header.stamp.nanosec = i * 5000000;
      sample->linear_acceleration.x = raw_acc.x();
      sample->linear_acceleration.y = raw_acc.y();
      sample->linear_acceleration.z = raw_acc.z();
      sample->angular_velocity.x = gyro_bias.x();
      sample->angular_velocity.y = gyro_bias.y();
      sample->angular_velocity.z = gyro_bias.z();
      measurements.imu.push_back(sample);
    }
    auto cloud = std::make_shared<PointCloudXYZI>();
    imu.Process(measurements, filter, cloud);

    input_ikfom input;
    input.acc = raw_acc * imu.get_acc_scale();
    input.gyro = gyro_bias;
    auto initialized = filter.get_x();
    const double residual = get_f(initialized, input).segment<3>(12).norm();
    const double gravity_error = (V3D(initialized.grav) - gravity_world).norm();
    auto noise = process_noise_cov();
    double dt = 0.01;
    for (int i = 0; i < 100; ++i) {
      filter.predict(dt, noise, input);
    }
    const auto propagated = filter.get_x();
    const double drift = (propagated.pos - initial.pos).norm();
    const double speed = propagated.vel.norm();
    // S2 fixes |g| at 9.809 while G_m_s2 is 9.81. Allow this existing
    // 0.001 m/s^2 mismatch, but reject attitude-dependent acceleration.
    const bool ok = residual < 0.002 && gravity_error < 0.002 &&
                    drift < 0.001 && speed < 0.002 && filter.get_P().allFinite();
    passed = passed && ok;
    std::cout << (ok ? "PASS " : "FAIL ") << name
              << " acceleration_residual=" << residual
              << " gravity_error=" << gravity_error
              << " drift_1s=" << drift << " speed_1s=" << speed << '\n';
  }
  return passed ? 0 : 1;
}
