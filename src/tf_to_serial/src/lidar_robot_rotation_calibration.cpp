#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr double MID360_IMU2LIDAR_Z = -0.04412;
constexpr double MID360_IMU2LIDAR_DIS = 0.025757;
constexpr double MID360_IMU2LIDAR_ANG = std::atan2(0.02329, 0.011);

double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

double wrap_to_pi(double rad) {
    rad = std::fmod(rad + M_PI, 2.0 * M_PI);
    if (rad < 0.0) rad += 2.0 * M_PI;
    return rad - M_PI;
}

double wrap_to_180(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg - 180.0;
}

double norm2(double x, double y) {
    return std::sqrt(x * x + y * y);
}

double circular_mean(double sin_sum, double cos_sum, double fallback) {
    if (std::fabs(sin_sum) < 1e-12 && std::fabs(cos_sum) < 1e-12) {
        return fallback;
    }
    return std::atan2(sin_sum, cos_sum);
}

bool solve_3x3(std::array<std::array<double, 4>, 3> a, std::array<double, 3>& x) {
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        double best = std::fabs(a[col][col]);
        for (int row = col + 1; row < 3; ++row) {
            const double v = std::fabs(a[row][col]);
            if (v > best) {
                best = v;
                pivot = row;
            }
        }
        if (best < 1e-12) return false;
        if (pivot != col) std::swap(a[pivot], a[col]);

        const double div = a[col][col];
        for (int k = col; k < 4; ++k) a[col][k] /= div;

        for (int row = 0; row < 3; ++row) {
            if (row == col) continue;
            const double factor = a[row][col];
            for (int k = col; k < 4; ++k) a[row][k] -= factor * a[col][k];
        }
    }
    x = {a[0][3], a[1][3], a[2][3]};
    return true;
}

}  // namespace

class LidarRobotRotationCalibration : public rclcpp::Node {
public:
    LidarRobotRotationCalibration() : Node("lidar_robot_rotation_calibration") {
        source_frame_ = declare_parameter<std::string>("source_frame", "camera_init");
        target_frame_ = declare_parameter<std::string>("target_frame", "body_hf");
        sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 50.0);
        warmup_frames_ = declare_parameter<int>("warmup_frames", 80);
        rotate_min_samples_ = declare_parameter<int>("rotate_min_samples", 500);
        rotate_max_samples_ = declare_parameter<int>("rotate_max_samples", 6000);
        rotate_min_yaw_span_deg_ = declare_parameter<double>("rotate_min_yaw_span_deg", 360.0);
        rotate_min_stable_windows_ = declare_parameter<int>("rotate_min_stable_windows", 3);
        rotate_radius_stable_m_ = declare_parameter<double>("rotate_radius_stable_m", 0.005);
        rotate_phase_stable_deg_ = declare_parameter<double>("rotate_phase_stable_deg", 1.0);
        rotate_max_radius_rms_m_ = declare_parameter<double>("rotate_max_radius_rms_m", 0.025);
        rotate_max_phase_rms_deg_ = declare_parameter<double>("rotate_max_phase_rms_deg", 4.0);
        rotate_target_phase_rms_deg_ = declare_parameter<double>("rotate_target_phase_rms_deg", 2.5);
        rotate_best_extra_samples_ = declare_parameter<int>("rotate_best_extra_samples", 300);
        rotate_radius_trim_ratio_ = declare_parameter<double>("rotate_radius_trim_ratio", 0.10);
        rotate_phase_trim_ratio_ = declare_parameter<double>("rotate_phase_trim_ratio", 0.10);
        rotate_start_min_yaw_deg_ = declare_parameter<double>("rotate_start_min_yaw_deg", 3.0);
        rotate_start_min_motion_m_ = declare_parameter<double>("rotate_start_min_motion_m", 0.02);
        gravity_fix_roll_deg_ = declare_parameter<double>("gravity_fix_roll_deg", 0.0);
        gravity_fix_pitch_deg_ =
            declare_parameter<double>("gravity_fix_pitch_deg", -(180.0 - 55.1425));
        gravity_fix_yaw_deg_ = declare_parameter<double>("gravity_fix_yaw_deg", 0.0);

        if (sample_rate_hz_ <= 0.0) sample_rate_hz_ = 50.0;
        if (warmup_frames_ < 1) warmup_frames_ = 1;
        if (rotate_min_samples_ < 50) rotate_min_samples_ = 50;
        if (rotate_max_samples_ < rotate_min_samples_) rotate_max_samples_ = rotate_min_samples_;
        if (rotate_best_extra_samples_ < 0) rotate_best_extra_samples_ = 0;
        rotate_radius_trim_ratio_ = std::clamp(rotate_radius_trim_ratio_, 0.0, 0.45);
        rotate_phase_trim_ratio_ = std::clamp(rotate_phase_trim_ratio_, 0.0, 0.45);

        q_fix_.setRPY(
            deg_to_rad(gravity_fix_roll_deg_),
            deg_to_rad(gravity_fix_pitch_deg_),
            deg_to_rad(gravity_fix_yaw_deg_));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto period = std::chrono::duration<double>(1.0 / sample_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&LidarRobotRotationCalibration::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Rotation calibration started.");
        RCLCPP_INFO(get_logger(),
            "Keep still for warmup, then rotate around robot center. Multiple turns are OK.");
    }

private:
    struct Sample {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct CircleFit {
        double cx = 0.0;
        double cy = 0.0;
        double radius = 0.0;
        double rms = 0.0;
        double raw_rms = 0.0;
        double phase = 0.0;
        double phase_rms = 0.0;
        double raw_phase_rms = 0.0;
        double score = std::numeric_limits<double>::infinity();
        std::size_t sample_count = 0;
        std::size_t radius_inliers = 0;
        std::size_t phase_inliers = 0;
        double path_span_deg = 0.0;
        double yaw_span_deg = 0.0;
    };

    void timer_callback() {
        try {
            const auto tf = tf_buffer_->lookupTransform(
                source_frame_, target_frame_, tf2::TimePointZero);
            process_transform(tf);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", ex.what());
        }
    }

    void process_transform(const geometry_msgs::msg::TransformStamped& tf) {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        if (!build_leveled_lidar_state(tf, x, y, yaw)) {
            return;
        }

        if (!capture_started_) {
            handle_wait_for_rotation_start(x, y, yaw);
            return;
        }

        samples_.push_back({x, y, yaw});

        CircleFit fit;
        const bool fit_ok = fit_circle(samples_, fit);
        if (fit_ok) {
            update_stability(fit);
        }

        const double yaw_span = yaw_span_deg();
        const double path_span = fit_ok ? path_span_deg(fit) : 0.0;
        if (fit_ok) {
            fit.path_span_deg = path_span;
            fit.yaw_span_deg = yaw_span;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "ROTATE: n=%zu/%d path_span=%.1f/%.1f deg yaw_span=%.1f deg stable=%d/%d%s",
            samples_.size(), rotate_max_samples_, path_span, rotate_min_yaw_span_deg_, yaw_span,
            stable_windows_, rotate_min_stable_windows_, fit_ok ? "" : " fit_wait");

        if (fit_ok) {
            const bool enough_samples = static_cast<int>(samples_.size()) >= rotate_min_samples_;
            const bool enough_path = path_span >= rotate_min_yaw_span_deg_;
            const bool quality_ok =
                fit.rms <= rotate_max_radius_rms_m_ &&
                rad_to_deg(fit.phase_rms) <= rotate_max_phase_rms_deg_;
            const bool stable_ok = stable_windows_ >= rotate_min_stable_windows_;
            if (enough_samples && enough_path && quality_ok) {
                update_best_fit(fit);
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "ROTATE gates: samples=%s path=%s quality=%s stable=%s "
                "(dis=%.4f m, phase=%.3f deg, radius_rms %.4f/%.4f m raw=%.4f m, "
                "phase_rms %.3f/%.3f deg raw=%.3f deg, inliers=%zu/%zu)",
                enough_samples ? "OK" : "WAIT",
                enough_path ? "OK" : "WAIT",
                quality_ok ? "OK" : "WAIT",
                stable_ok ? "OK" : "WAIT",
                fit.radius, wrap_to_180(rad_to_deg(fit.phase)),
                fit.rms, rotate_max_radius_rms_m_, fit.raw_rms,
                rad_to_deg(fit.phase_rms), rotate_max_phase_rms_deg_,
                rad_to_deg(fit.raw_phase_rms), fit.phase_inliers, fit.sample_count);

            if (std::fabs(path_span - yaw_span) > 180.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Trajectory span and TF yaw span differ a lot: path_span=%.1f deg, yaw_span=%.1f deg. "
                    "If this persists, the TF yaw source is not tracking physical rotation reliably.",
                    path_span, yaw_span);
            }

            if (have_best_fit_ && stable_ok && ready_to_finish_best_fit()) {
                print_result(best_fit_);
                rclcpp::shutdown();
                return;
            }
        }

        if (static_cast<int>(samples_.size()) >= rotate_max_samples_) {
            if (have_best_fit_) {
                RCLCPP_WARN(get_logger(), "Max samples reached; using best robust rotation fit.");
                print_result(best_fit_);
            } else if (fit_ok) {
                RCLCPP_WARN(get_logger(), "Max samples reached; using current robust rotation fit.");
                print_result(fit);
            } else {
                RCLCPP_ERROR(get_logger(),
                    "Rotation fit failed. Check whether robot actually rotates around its center.");
            }
            rclcpp::shutdown();
        }
    }

    bool build_leveled_lidar_state(
        const geometry_msgs::msg::TransformStamped& tf,
        double& out_x,
        double& out_y,
        double& out_yaw) {
        const double raw_x = tf.transform.translation.x;
        const double raw_y = tf.transform.translation.y;
        const double raw_z = tf.transform.translation.z;

        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);

        tf2::Quaternion q_leveled = q_fix_ * q;
        q_leveled.normalize();

        double roll_leveled = 0.0;
        double pitch_leveled = 0.0;
        double yaw_leveled = 0.0;
        tf2::Matrix3x3(q_leveled).getRPY(roll_leveled, pitch_leveled, yaw_leveled);

        if (!origin_ready_) {
            sum_x_ += raw_x;
            sum_y_ += raw_y;
            sum_z_ += raw_z;
            sum_yaw_sin_ += std::sin(yaw_leveled);
            sum_yaw_cos_ += std::cos(yaw_leveled);
            ++warmup_count_;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "Warming up origin... %d/%d", warmup_count_, warmup_frames_);
            if (warmup_count_ < warmup_frames_) {
                return false;
            }

            avg_x_ = sum_x_ / warmup_frames_;
            avg_y_ = sum_y_ / warmup_frames_;
            avg_z_ = sum_z_ / warmup_frames_;
            avg_yaw_leveled_ = circular_mean(sum_yaw_sin_, sum_yaw_cos_, yaw_leveled);
            origin_ready_ = true;
            RCLCPP_WARN(get_logger(), "Warmup done. Start rotation when ready.");
            return false;
        }

        const double x = raw_x - avg_x_;
        const double y = raw_y - avg_y_;
        const double z = raw_z - avg_z_;

        const tf2::Vector3 t_li(
            MID360_IMU2LIDAR_DIS * std::cos(MID360_IMU2LIDAR_ANG),
            MID360_IMU2LIDAR_DIS * std::sin(MID360_IMU2LIDAR_ANG),
            MID360_IMU2LIDAR_Z);
        const tf2::Vector3 correction = tf2::quatRotate(q, t_li);

        const double x_imu2lidar_local = x - correction.x();
        const double y_imu2lidar_local = y - correction.y();
        const double z_imu2lidar_local = z - correction.z();

        tf2::Vector3 p_vec(x_imu2lidar_local, y_imu2lidar_local, z_imu2lidar_local);
        p_vec = tf2::quatRotate(q_fix_, p_vec);

        out_x = p_vec.x();
        out_y = p_vec.y();
        out_yaw = unwrap_yaw(yaw_leveled - avg_yaw_leveled_);
        return true;
    }

    void handle_wait_for_rotation_start(double x, double y, double yaw) {
        if (!have_start_reference_) {
            start_ref_x_ = x;
            start_ref_y_ = y;
            start_ref_yaw_ = yaw;
            have_start_reference_ = true;
            RCLCPP_WARN(get_logger(), "Waiting for rotation to start...");
            return;
        }

        const double motion = norm2(x - start_ref_x_, y - start_ref_y_);
        const double yaw_delta_deg = std::fabs(rad_to_deg(wrap_to_pi(yaw - start_ref_yaw_)));
        const bool moved_enough =
            motion >= rotate_start_min_motion_m_ || yaw_delta_deg >= rotate_start_min_yaw_deg_;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "WAIT_ROTATION: motion=%.3f/%.3f m yaw_delta=%.2f/%.2f deg",
            motion, rotate_start_min_motion_m_, yaw_delta_deg, rotate_start_min_yaw_deg_);

        if (!moved_enough) {
            return;
        }

        capture_started_ = true;
        samples_.clear();
        have_last_fit_ = false;
        have_best_fit_ = false;
        first_quality_sample_ = 0;
        stable_windows_ = 0;
        samples_.push_back({start_ref_x_, start_ref_y_, start_ref_yaw_});
        samples_.push_back({x, y, yaw});
        RCLCPP_WARN(get_logger(), "Rotation detected. Collecting circle samples now.");
    }

    double unwrap_yaw(double yaw) {
        if (!have_last_yaw_) {
            have_last_yaw_ = true;
            last_yaw_wrapped_ = yaw;
            yaw_unwrapped_ = yaw;
            return yaw_unwrapped_;
        }

        const double delta = wrap_to_pi(yaw - last_yaw_wrapped_);
        yaw_unwrapped_ += delta;
        last_yaw_wrapped_ = yaw;
        return yaw_unwrapped_;
    }

    double yaw_span_deg() const {
        if (samples_.empty()) return 0.0;
        double min_yaw = samples_.front().yaw;
        double max_yaw = samples_.front().yaw;
        for (const auto& s : samples_) {
            min_yaw = std::min(min_yaw, s.yaw);
            max_yaw = std::max(max_yaw, s.yaw);
        }
        return rad_to_deg(max_yaw - min_yaw);
    }

    double path_span_deg(const CircleFit& fit) const {
        if (samples_.empty()) return 0.0;

        bool have_last = false;
        double last_phi = 0.0;
        double phi_unwrapped = 0.0;
        double min_phi = 0.0;
        double max_phi = 0.0;

        for (const auto& s : samples_) {
            const double phi = std::atan2(s.y - fit.cy, s.x - fit.cx);
            if (!have_last) {
                have_last = true;
                last_phi = phi;
                phi_unwrapped = phi;
                min_phi = phi;
                max_phi = phi;
                continue;
            }

            const double delta = wrap_to_pi(phi - last_phi);
            phi_unwrapped += delta;
            last_phi = phi;
            min_phi = std::min(min_phi, phi_unwrapped);
            max_phi = std::max(max_phi, phi_unwrapped);
        }

        return rad_to_deg(max_phi - min_phi);
    }

    bool fit_circle_least_squares(
        const std::vector<Sample>& samples,
        const std::vector<std::size_t>* indices,
        CircleFit& fit) const {
        const std::size_t count = indices ? indices->size() : samples.size();
        if (count < 20) return false;

        double sx = 0.0, sy = 0.0, sx2 = 0.0, sy2 = 0.0, sxy = 0.0;
        double sr2 = 0.0, sxr2 = 0.0, syr2 = 0.0;

        for (std::size_t i = 0; i < count; ++i) {
            const auto& s = indices ? samples[(*indices)[i]] : samples[i];
            const double x2 = s.x * s.x;
            const double y2 = s.y * s.y;
            const double r2 = x2 + y2;
            sx += s.x;
            sy += s.y;
            sx2 += x2;
            sy2 += y2;
            sxy += s.x * s.y;
            sr2 += r2;
            sxr2 += s.x * r2;
            syr2 += s.y * r2;
        }

        const double n = static_cast<double>(count);
        std::array<std::array<double, 4>, 3> a = {{
            {{sx2, sxy, sx, -sxr2}},
            {{sxy, sy2, sy, -syr2}},
            {{sx,  sy,  n,  -sr2}},
        }};

        std::array<double, 3> abc{};
        if (!solve_3x3(a, abc)) return false;

        const double radius_sq = 0.25 * (abc[0] * abc[0] + abc[1] * abc[1]) - abc[2];
        if (radius_sq <= 0.0 || !std::isfinite(radius_sq)) return false;

        fit.cx = -0.5 * abc[0];
        fit.cy = -0.5 * abc[1];
        fit.radius = std::sqrt(radius_sq);
        return true;
    }

    double quantile_threshold(std::vector<double> values, double trim_ratio) const {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        const double keep_ratio = std::clamp(1.0 - trim_ratio, 0.5, 1.0);
        std::size_t keep_count = static_cast<std::size_t>(
            std::ceil(keep_ratio * static_cast<double>(values.size())));
        keep_count = std::clamp<std::size_t>(keep_count, 1, values.size());
        return values[keep_count - 1];
    }

    void select_radius_inliers(
        const std::vector<Sample>& samples,
        const CircleFit& fit,
        std::vector<std::size_t>& inliers) const {
        std::vector<double> residual_abs;
        residual_abs.reserve(samples.size());
        for (const auto& s : samples) {
            const double r = norm2(s.x - fit.cx, s.y - fit.cy);
            residual_abs.push_back(std::fabs(r - fit.radius));
        }

        const double threshold = quantile_threshold(residual_abs, rotate_radius_trim_ratio_);
        inliers.clear();
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const double r = norm2(samples[i].x - fit.cx, samples[i].y - fit.cy);
            if (std::fabs(r - fit.radius) <= threshold) {
                inliers.push_back(i);
            }
        }
    }

    void compute_radius_quality(
        const std::vector<Sample>& samples,
        const std::vector<std::size_t>& inliers,
        CircleFit& fit) const {
        double raw_sq = 0.0;
        for (const auto& s : samples) {
            const double r = norm2(s.x - fit.cx, s.y - fit.cy);
            raw_sq += (r - fit.radius) * (r - fit.radius);
        }
        fit.raw_rms = std::sqrt(raw_sq / static_cast<double>(samples.size()));

        double robust_sq = 0.0;
        for (const auto idx : inliers) {
            const auto& s = samples[idx];
            const double r = norm2(s.x - fit.cx, s.y - fit.cy);
            robust_sq += (r - fit.radius) * (r - fit.radius);
        }
        fit.radius_inliers = inliers.size();
        fit.rms = inliers.empty()
            ? fit.raw_rms
            : std::sqrt(robust_sq / static_cast<double>(inliers.size()));
    }

    void compute_phase_quality(
        const std::vector<Sample>& samples,
        const std::vector<std::size_t>& radius_inliers,
        CircleFit& fit) const {
        std::vector<double> phases;
        phases.reserve(radius_inliers.empty() ? samples.size() : radius_inliers.size());

        if (radius_inliers.empty()) {
            for (const auto& s : samples) {
                phases.push_back(wrap_to_pi(std::atan2(s.y - fit.cy, s.x - fit.cx) - s.yaw));
            }
        } else {
            for (const auto idx : radius_inliers) {
                const auto& s = samples[idx];
                phases.push_back(wrap_to_pi(std::atan2(s.y - fit.cy, s.x - fit.cx) - s.yaw));
            }
        }

        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (const double phase : phases) {
            sin_sum += std::sin(phase);
            cos_sum += std::cos(phase);
        }
        const double raw_phase = std::atan2(sin_sum, cos_sum);

        std::vector<double> phase_errors;
        phase_errors.reserve(phases.size());
        double raw_sq = 0.0;
        for (const double phase : phases) {
            const double e = wrap_to_pi(phase - raw_phase);
            phase_errors.push_back(std::fabs(e));
            raw_sq += e * e;
        }
        fit.raw_phase_rms = std::sqrt(raw_sq / static_cast<double>(phases.size()));

        const double threshold = quantile_threshold(phase_errors, rotate_phase_trim_ratio_);
        sin_sum = 0.0;
        cos_sum = 0.0;
        fit.phase_inliers = 0;
        for (std::size_t i = 0; i < phases.size(); ++i) {
            if (phase_errors[i] <= threshold) {
                sin_sum += std::sin(phases[i]);
                cos_sum += std::cos(phases[i]);
                ++fit.phase_inliers;
            }
        }

        if (fit.phase_inliers < 20) {
            fit.phase = raw_phase;
            fit.phase_rms = fit.raw_phase_rms;
            fit.phase_inliers = phases.size();
            return;
        }

        fit.phase = std::atan2(sin_sum, cos_sum);
        double robust_sq = 0.0;
        for (std::size_t i = 0; i < phases.size(); ++i) {
            if (phase_errors[i] <= threshold) {
                const double e = wrap_to_pi(phases[i] - fit.phase);
                robust_sq += e * e;
            }
        }
        fit.phase_rms = std::sqrt(robust_sq / static_cast<double>(fit.phase_inliers));
    }

    bool fit_circle(const std::vector<Sample>& samples, CircleFit& fit) const {
        if (samples.size() < 20) return false;

        CircleFit initial;
        if (!fit_circle_least_squares(samples, nullptr, initial)) return false;

        std::vector<std::size_t> radius_inliers;
        select_radius_inliers(samples, initial, radius_inliers);
        if (radius_inliers.size() >= 20 && radius_inliers.size() < samples.size()) {
            if (!fit_circle_least_squares(samples, &radius_inliers, fit)) {
                fit = initial;
            }
        } else {
            fit = initial;
        }

        select_radius_inliers(samples, fit, radius_inliers);
        compute_radius_quality(samples, radius_inliers, fit);
        compute_phase_quality(samples, radius_inliers, fit);
        fit.sample_count = samples.size();
        fit.score =
            fit.rms / std::max(rotate_max_radius_rms_m_, 1e-9) +
            rad_to_deg(fit.phase_rms) / std::max(rotate_max_phase_rms_deg_, 1e-9);
        return true;
    }

    void update_best_fit(const CircleFit& fit) {
        if (!have_best_fit_ || fit.score < best_fit_.score) {
            best_fit_ = fit;
            have_best_fit_ = true;
        }
        if (first_quality_sample_ == 0) {
            first_quality_sample_ = samples_.size();
        }
    }

    bool ready_to_finish_best_fit() const {
        const bool target_ok =
            rad_to_deg(best_fit_.phase_rms) <= rotate_target_phase_rms_deg_;
        const bool collected_extra =
            first_quality_sample_ > 0 &&
            samples_.size() >= first_quality_sample_ + static_cast<std::size_t>(rotate_best_extra_samples_);
        return target_ok || collected_extra;
    }

    void update_stability(const CircleFit& fit) {
        if (!have_last_fit_) {
            last_fit_ = fit;
            have_last_fit_ = true;
            return;
        }

        const double radius_delta = std::fabs(fit.radius - last_fit_.radius);
        const double phase_delta = std::fabs(rad_to_deg(wrap_to_pi(fit.phase - last_fit_.phase)));
        if (radius_delta <= rotate_radius_stable_m_ && phase_delta <= rotate_phase_stable_deg_) {
            ++stable_windows_;
        } else {
            stable_windows_ = 0;
        }
        last_fit_ = fit;
    }

    void print_result(const CircleFit& fit) const {
        RCLCPP_INFO(get_logger(), "========== ROTATION CALIBRATION RESULTS ==========");
        RCLCPP_INFO(get_logger(), "samples=%zu path_span=%.2f deg yaw_span=%.2f deg",
            fit.sample_count, fit.path_span_deg, fit.yaw_span_deg);
        RCLCPP_INFO(get_logger(), "center_x=%.6f m center_y=%.6f m", fit.cx, fit.cy);
        RCLCPP_INFO(get_logger(), "lidar2robot_dis=%.6f m", fit.radius);
        RCLCPP_INFO(get_logger(), "observable_phase_deg=%.6f", wrap_to_180(rad_to_deg(fit.phase)));
        RCLCPP_INFO(get_logger(),
            "quality: radius_rms=%.6f m phase_rms=%.6f deg raw_radius_rms=%.6f m raw_phase_rms=%.6f deg",
            fit.rms, rad_to_deg(fit.phase_rms), fit.raw_rms, rad_to_deg(fit.raw_phase_rms));
        RCLCPP_INFO(get_logger(), "inliers: radius=%zu/%zu phase=%zu/%zu",
            fit.radius_inliers, fit.sample_count, fit.phase_inliers, fit.sample_count);
        RCLCPP_INFO(get_logger(), "Use observable_phase_deg as input for lidar_robot_line_calibration.");
        RCLCPP_INFO(get_logger(), "Recommended code parameter:");
        RCLCPP_INFO(get_logger(), "  lidar2robot_dis_ = %.6f;", fit.radius);
        RCLCPP_INFO(get_logger(), "==================================================");
    }

    std::string source_frame_;
    std::string target_frame_;
    double sample_rate_hz_ = 50.0;
    int warmup_frames_ = 80;
    int rotate_min_samples_ = 500;
    int rotate_max_samples_ = 6000;
    double rotate_min_yaw_span_deg_ = 360.0;
    int rotate_min_stable_windows_ = 3;
    double rotate_radius_stable_m_ = 0.005;
    double rotate_phase_stable_deg_ = 1.0;
    double rotate_max_radius_rms_m_ = 0.025;
    double rotate_max_phase_rms_deg_ = 4.0;
    double rotate_target_phase_rms_deg_ = 2.5;
    int rotate_best_extra_samples_ = 300;
    double rotate_radius_trim_ratio_ = 0.10;
    double rotate_phase_trim_ratio_ = 0.10;
    double rotate_start_min_yaw_deg_ = 3.0;
    double rotate_start_min_motion_m_ = 0.02;
    double gravity_fix_roll_deg_ = 0.0;
    double gravity_fix_pitch_deg_ = 0.0;
    double gravity_fix_yaw_deg_ = 0.0;

    tf2::Quaternion q_fix_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    int warmup_count_ = 0;
    bool origin_ready_ = false;
    double sum_x_ = 0.0;
    double sum_y_ = 0.0;
    double sum_z_ = 0.0;
    double sum_yaw_sin_ = 0.0;
    double sum_yaw_cos_ = 0.0;
    double avg_x_ = 0.0;
    double avg_y_ = 0.0;
    double avg_z_ = 0.0;
    double avg_yaw_leveled_ = 0.0;

    bool have_last_yaw_ = false;
    double last_yaw_wrapped_ = 0.0;
    double yaw_unwrapped_ = 0.0;

    std::vector<Sample> samples_;
    CircleFit last_fit_;
    CircleFit best_fit_;
    bool have_last_fit_ = false;
    bool have_best_fit_ = false;
    std::size_t first_quality_sample_ = 0;
    int stable_windows_ = 0;
    bool have_start_reference_ = false;
    bool capture_started_ = false;
    double start_ref_x_ = 0.0;
    double start_ref_y_ = 0.0;
    double start_ref_yaw_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarRobotRotationCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
