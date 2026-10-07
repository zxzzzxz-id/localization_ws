#include <cmath>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <cassert>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <ros2_utils.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "use-ikfom.hpp"
#include "preprocess.h"
#include "posebuffer.h"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void set_initial_lidar_in_imu(const V3D &translation, const M3D &rotation);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void configure_orientation_observation(bool enabled, const V3D &stddev_rad,
                                         double gate_chi2,
                                         bool use_message_covariance,
                                         const M3D &R_external_from_internal);
  void ResetOrientationReference();
  void RebaseWorldFrame(const M3D &rotation_new_from_old);

  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  // Accelerometer G-scaling applied to every measurement before propagation
  // (see the `acc_avr = acc_avr * G_m_s2 / mean_acc.norm();` step below).
  // Exposed so the high-frequency odometry integrator uses the same scale.
  double get_acc_scale() const { return G_m_s2 / mean_acc.norm(); }


  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int lidar_type;
  PoseBuffer pbuffer;

 private:
  double gap_handler(double dt) const;
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);
  bool UpdateOrientationObservation(const ImuMsgConstPtr &imu,
      esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);

  PointCloudXYZI::Ptr cur_pcl_un_;
  ImuMsgConstPtr last_imu_;
  deque<ImuMsgConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D R_lidar_in_imu;
  V3D t_lidar_in_imu;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  double imu_gap_ = 0.5;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
  bool   orientation_observation_enabled_ = false;
  bool   orientation_reference_ready_ = false;
  bool   orientation_use_message_covariance_ = false;
  V3D    orientation_stddev_rad_ = V3D(0.0523598776, 0.0523598776, 0.1745329252);
  double orientation_gate_chi2_ = 16.27;
  M3D    R_odom_from_navigation_ = Eye3d;
  M3D    R_external_from_internal_ = Eye3d;
  std::size_t orientation_rejection_count_ = 0;
};

ImuProcess::ImuProcess()
    : start_timestamp_(-1), last_lidar_end_time_(-1),
      b_first_frame_(true), imu_need_init_(true)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last   = Zero3d;
  acc_s_last    = Zero3d;
  t_lidar_in_imu = Zero3d;
  R_lidar_in_imu = Eye3d;
  last_imu_.reset(new ImuMsg());
}

ImuProcess::~ImuProcess() {}

double ImuProcess::gap_handler(double dt) const
{
  if (dt < 0.0 || dt > imu_gap_)
    return -1.0;
  return dt;
}

void ImuProcess::RebaseWorldFrame(const M3D &rotation_new_from_old)
{
  // These cached quantities are expressed in the world frame. Keep them in
  // the same frame as the rebased EKF state before processing the next scan.
  acc_s_last = rotation_new_from_old * acc_s_last;
  if (orientation_reference_ready_) {
    R_odom_from_navigation_ = rotation_new_from_old * R_odom_from_navigation_;
  }
  IMUpose.clear();
  pbuffer.Clear();
}

void ImuProcess::configure_orientation_observation(
    bool enabled, const V3D &stddev_rad, double gate_chi2,
    bool use_message_covariance, const M3D &R_external_from_internal)
{
  orientation_observation_enabled_ = enabled;
  orientation_stddev_rad_ = stddev_rad;
  orientation_gate_chi2_ = gate_chi2;
  orientation_use_message_covariance_ = use_message_covariance;
  R_external_from_internal_ = R_external_from_internal;
}

void ImuProcess::ResetOrientationReference()
{
  orientation_reference_ready_ = false;
  R_odom_from_navigation_ = Eye3d;
  orientation_rejection_count_ = 0;
}


void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last   = Zero3d;
  acc_s_last    = Zero3d;
  imu_need_init_    = true;
  b_first_frame_    = true;
  start_timestamp_  = -1;
  last_lidar_end_time_ = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  pbuffer.Clear();
  ResetOrientationReference();
  last_imu_.reset(new ImuMsg());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_initial_lidar_in_imu(const V3D &translation, const M3D &rotation)
{
  t_lidar_in_imu = translation;
  R_lidar_in_imu = rotation;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

bool ImuProcess::UpdateOrientationObservation(
    const ImuMsgConstPtr &imu,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state)
{
  if (!orientation_observation_enabled_ || imu->orientation_covariance[0] < 0.0) return false;
  Eigen::Quaterniond q_navigation_from_imu(
      imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  if (!q_navigation_from_imu.coeffs().allFinite() ||
      q_navigation_from_imu.squaredNorm() < 1e-12) return false;
  q_navigation_from_imu.normalize();

  const state_ikfom &current_state = kf_state.get_x();
  const M3D R_odom_from_imu = current_state.rot.toRotationMatrix();
  const M3D R_navigation_from_imu = q_navigation_from_imu.toRotationMatrix() * R_external_from_internal_;
  if (!orientation_reference_ready_) {
    // AHRS is body -> navigation (ENU); FAST-LIO owns an arbitrary odom frame.
    R_odom_from_navigation_ = R_odom_from_imu * R_navigation_from_imu.transpose();
    orientation_reference_ready_ = true;
    ROS_PRINT_INFO("IMU orientation observation reference initialized (navigation -> odom).");
    return false;
  }

  const M3D R_measurement = R_odom_from_navigation_ * R_navigation_from_imu;
  const V3D residual = Log((R_odom_from_imu.transpose() * R_measurement).eval());
  const auto rotation_to_rpy_deg = [](const M3D &rotation) {
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -rotation(2, 0))));
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    constexpr double kRadToDeg = 180.0 / M_PI;
    return V3D(roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg);
  };
  const auto maybe_log_orientation = [&](const char *status, double nis,
                                          const M3D &filter_rotation) {
    static std::chrono::steady_clock::time_point last_log_time;
    const auto now = std::chrono::steady_clock::now();
    if (last_log_time.time_since_epoch().count() != 0 &&
        now - last_log_time < std::chrono::seconds(1))
      return;
    last_log_time = now;
    const V3D raw_rpy_deg = rotation_to_rpy_deg(q_navigation_from_imu.toRotationMatrix());
    const V3D transformed_rpy_deg = rotation_to_rpy_deg(R_measurement);
    const V3D filter_rpy_deg = rotation_to_rpy_deg(filter_rotation);
    ROS_PRINT_INFO(
        "IMU RPY compare [%s] raw(nav)=(%.2f, %.2f, %.2f) deg "
        "transformed(odom)=(%.2f, %.2f, %.2f) deg "
        "fastlio(odom->imu)=(%.2f, %.2f, %.2f) deg NIS=%.3f",
        status,
        raw_rpy_deg.x(), raw_rpy_deg.y(), raw_rpy_deg.z(),
        transformed_rpy_deg.x(), transformed_rpy_deg.y(), transformed_rpy_deg.z(),
        filter_rpy_deg.x(), filter_rpy_deg.y(), filter_rpy_deg.z(), nis);
  };
  M3D measurement_covariance = orientation_stddev_rad_.array().square().matrix().asDiagonal();
  if (orientation_use_message_covariance_) {
    M3D message_covariance;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col)
        message_covariance(row, col) = imu->orientation_covariance[row * 3 + col];
    }
    Eigen::LDLT<M3D> message_covariance_ldlt(message_covariance);
    if (message_covariance.allFinite() && message_covariance_ldlt.info() == Eigen::Success &&
        (message_covariance.diagonal().array() > 0.0).all())
      measurement_covariance = message_covariance;
  }

  using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
  Filter::cov covariance = kf_state.get_P();
  Eigen::Matrix<double, 3, state_ikfom::DOF> H =
      Eigen::Matrix<double, 3, state_ikfom::DOF>::Zero();
  H.block<3, 3>(0, 3).setIdentity();
  const M3D innovation_covariance = H * covariance * H.transpose() + measurement_covariance;
  const Eigen::LDLT<M3D> innovation_solver(innovation_covariance);
  if (innovation_solver.info() != Eigen::Success) return false;
  const double nis = residual.dot(innovation_solver.solve(residual));
  if (!std::isfinite(nis) || nis > orientation_gate_chi2_) {
    ++orientation_rejection_count_;
    maybe_log_orientation("rejected", nis, R_odom_from_imu);
    if (orientation_rejection_count_ == 1 || orientation_rejection_count_ % 200 == 0)
      ROS_PRINT_WARN("Rejected IMU orientation: NIS=%.3f gate=%.3f (rejected=%zu).",
                     nis, orientation_gate_chi2_, orientation_rejection_count_);
    return false;
  }

  const Eigen::Matrix<double, state_ikfom::DOF, 3> kalman_gain =
      covariance * H.transpose() * innovation_solver.solve(M3D::Identity());
  Filter::vectorized_state correction = kalman_gain * residual;
  state_ikfom corrected_state = current_state;
  corrected_state.boxplus(correction);
  const Filter::cov I_KH = Filter::cov::Identity() - kalman_gain * H;
  Filter::cov corrected_covariance =
      I_KH * covariance * I_KH.transpose() +
      kalman_gain * measurement_covariance * kalman_gain.transpose();
  corrected_covariance = 0.5 * (corrected_covariance + corrected_covariance.transpose());
  kf_state.change_x(corrected_state);
  kf_state.change_P(corrected_covariance);
  maybe_log_orientation("accepted", nis, corrected_state.rot.toRotationMatrix());
  return true;
}


void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  // Initial T_lidar^imu. FAST-LIO may update these entries online when the
  // measurement Jacobian enables its extrinsic columns.
  init_state.t_lidar_in_imu = t_lidar_in_imu;
  init_state.R_lidar_in_imu = R_lidar_in_imu;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = meas.lidar_end_time;

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double imu_beg_time = get_ros_time_sec(v_imu.front()->header.stamp);
  const double imu_end_time = get_ros_time_sec(v_imu.back()->header.stamp);

  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;

    if (lidar_type == MARSIM) {
        pcl_beg_time = last_lidar_end_time_;
        pcl_end_time = meas.lidar_beg_time;
    }

    /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;
  std::size_t orientation_index = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    const double head_time = get_ros_time_sec(head->header.stamp);
    const double tail_time = get_ros_time_sec(tail->header.stamp);

    if (head_time < last_lidar_end_time_) continue;

    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head_time - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head_time < last_lidar_end_time_)
    {
      dt = tail_time - last_lidar_end_time_;
      // dt = tail_time - pcl_beg_time;
    }
    else
    {
      dt = tail_time - head_time;
    }

    dt = gap_handler(dt);
    if (dt < 0.0)
      continue;

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);
    while (orientation_index < meas.orientation.size() &&
           get_ros_time_sec(meas.orientation[orientation_index]->header.stamp) <= tail_time) {
      UpdateOrientationObservation(meas.orientation[orientation_index++], kf_state);
    }

    /* save the pose at each IMU measurement for undistortion */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    const double offs_t = tail_time - last_lidar_end_time_;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  dt = gap_handler(dt);
  if (dt < 0.0)
  {
    last_imu_ = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;
    return;
  }
  kf_state.predict(dt, Q, in);
  while (orientation_index < meas.orientation.size()) {
    UpdateOrientationObservation(meas.orientation[orientation_index++], kf_state);
  }

  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;

  if(lidar_type != MARSIM){
      auto it_pcl = pcl_out.points.end() - 1;
      for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
      {
          auto head = it_kp - 1;
          auto tail = it_kp;
          R_imu<<MAT_FROM_ARRAY(head->rot);
          // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
          vel_imu<<VEC_FROM_ARRAY(head->vel);
          pos_imu<<VEC_FROM_ARRAY(head->pos);
          acc_imu<<VEC_FROM_ARRAY(tail->acc);
          angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

          for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
          {
              dt = it_pcl->curvature / double(1000) - head->offset_time;

              /* Transform to the 'end' frame, using only the rotation
               * Note: Compensation direction is INVERSE of Frame's moving direction
               * So if we want to compensate a point at timestamp-i to the frame-e
               * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
              M3D R_i(R_imu * Exp(angvel_avr, dt));

              V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
              V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
              V3D P_compensate = imu_state.R_lidar_in_imu.conjugate() *
                  (imu_state.rot.conjugate() *
                   (R_i * (imu_state.R_lidar_in_imu * P_i + imu_state.t_lidar_in_imu) + T_ei) -
                   imu_state.t_lidar_in_imu);

              // save Undistorted points and their rotation
              it_pcl->x = P_compensate(0);
              it_pcl->y = P_compensate(1);
              it_pcl->z = P_compensate(2);

              if (it_pcl == pcl_out.points.begin()) break;
          }
      }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;

      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
