#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double MID360_IMU2LIDAR_X = 0.011;
constexpr double MID360_IMU2LIDAR_Y = 0.02329;
constexpr double MID360_IMU2LIDAR_Z = -0.04412;

double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

double norm2(double x, double y) {
    return std::sqrt(x * x + y * y);
}

}  // namespace

class TiltDynamicCalibration : public rclcpp::Node {
public:
    TiltDynamicCalibration() : Node("tilt_dynamic_calibration") {
        source_frame_ = declare_parameter<std::string>("source_frame", "camera_init");
        target_frame_ = declare_parameter<std::string>("target_frame", "body_hf");
        sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 80.0);
        warmup_frames_ = declare_parameter<int>("warmup_frames", 120);
        move_min_samples_ = declare_parameter<int>("move_min_samples", 200);
        move_max_samples_ = declare_parameter<int>("move_max_samples", 3000);
        move_min_distance_m_ = declare_parameter<double>("move_min_distance_m", 0.8);
        start_min_motion_m_ = declare_parameter<double>("start_min_motion_m", 0.03);
        stop_window_samples_ = declare_parameter<int>("stop_window_samples", 80);
        stop_motion_m_ = declare_parameter<double>("stop_motion_m", 0.02);

        seed_fix_roll_deg_ = declare_parameter<double>("seed_fix_roll_deg", 0.0);
        seed_fix_pitch_deg_ = declare_parameter<double>("seed_fix_pitch_deg", -56.39);
        seed_fix_yaw_deg_ = declare_parameter<double>("seed_fix_yaw_deg", 0.0);
        search_roll_deg_ = declare_parameter<double>("search_roll_deg", 6.0);
        search_pitch_deg_ = declare_parameter<double>("search_pitch_deg", 6.0);
        search_step_deg_ = declare_parameter<double>("search_step_deg", 1.0);
        optimizer_rounds_ = declare_parameter<int>("optimizer_rounds", 5);

        if (sample_rate_hz_ <= 0.0) sample_rate_hz_ = 80.0;
        if (warmup_frames_ < 10) warmup_frames_ = 10;
        if (move_min_samples_ < 20) move_min_samples_ = 20;
        if (move_max_samples_ < move_min_samples_) move_max_samples_ = move_min_samples_;
        if (stop_window_samples_ < 5) stop_window_samples_ = 5;
        if (search_step_deg_ <= 0.0) search_step_deg_ = 1.0;
        if (optimizer_rounds_ < 1) optimizer_rounds_ = 1;

        seed_fix_.setRPY(
            deg_to_rad(seed_fix_roll_deg_),
            deg_to_rad(seed_fix_pitch_deg_),
            deg_to_rad(seed_fix_yaw_deg_));
        seed_fix_.normalize();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto period = std::chrono::duration<double>(1.0 / sample_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TiltDynamicCalibration::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Dynamic tilt calibration started.");
        RCLCPP_INFO(get_logger(), "Keep still for warmup, then move on flat ground and stop.");
        RCLCPP_INFO(get_logger(), "Seed fix_rpy_deg: %.3f %.3f %.3f",
            seed_fix_roll_deg_, seed_fix_pitch_deg_, seed_fix_yaw_deg_);
    }

private:
    struct RawSample {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        tf2::Quaternion q;
    };

    struct Metrics {
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double z_rms = std::numeric_limits<double>::infinity();
        double z_max_abs = std::numeric_limits<double>::infinity();
        double z_peak_to_peak = std::numeric_limits<double>::infinity();
        double z_slope_abs = std::numeric_limits<double>::infinity();
        double xy_distance = 0.0;
        double score = std::numeric_limits<double>::infinity();
        bool valid = false;
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
        const rclcpp::Time stamp(tf.header.stamp);
        if (stamp.nanoseconds() != 0 && have_last_tf_stamp_ && stamp == last_tf_stamp_) {
            return;
        }
        if (stamp.nanoseconds() != 0) {
            have_last_tf_stamp_ = true;
            last_tf_stamp_ = stamp;
        }

        RawSample sample;
        sample.x = tf.transform.translation.x;
        sample.y = tf.transform.translation.y;
        sample.z = tf.transform.translation.z;
        sample.q = tf2::Quaternion(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);

        if (warmup_samples_.size() < static_cast<std::size_t>(warmup_frames_)) {
            warmup_samples_.push_back(sample);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "WARMUP: keep still... %zu/%d", warmup_samples_.size(), warmup_frames_);
            if (warmup_samples_.size() == static_cast<std::size_t>(warmup_frames_)) {
                compute_seed_origin();
                RCLCPP_WARN(get_logger(), "Warmup done. Move on flat ground now.");
            }
            return;
        }

        const tf2::Vector3 seed_p = corrected_point(seed_fix_, sample, seed_origin_);
        if (!capture_started_) {
            wait_for_motion_start(sample, seed_p);
            return;
        }

        move_samples_.push_back(sample);
        seed_xy_history_.push_back({seed_p.x(), seed_p.y()});

        const double distance = seed_motion_distance();
        const bool enough_samples =
            static_cast<int>(move_samples_.size()) >= move_min_samples_;
        const bool enough_distance = distance >= move_min_distance_m_;
        const bool stopped = motion_stopped();
        const bool forced_stop =
            static_cast<int>(move_samples_.size()) >= move_max_samples_;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "MOVE_CAPTURE: fresh_tf_samples=%zu/%d distance=%.3f/%.3f m stopped=%s",
            move_samples_.size(), move_max_samples_, distance, move_min_distance_m_,
            stopped ? "yes" : "no");

        if ((enough_samples && enough_distance && stopped) || forced_stop) {
            if (forced_stop) {
                RCLCPP_WARN(get_logger(), "Max samples reached; optimizing with collected movement data.");
            }
            const Metrics best = optimize();
            print_result(best);
            rclcpp::shutdown();
        }
    }

    struct Origin {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    void compute_seed_origin() {
        seed_origin_ = compute_origin(warmup_samples_);
    }

    Origin compute_origin(const std::vector<RawSample>& samples) const {
        Origin origin;
        for (const auto& s : samples) {
            origin.x += s.x;
            origin.y += s.y;
            origin.z += s.z;
        }
        const double n = static_cast<double>(samples.size());
        origin.x /= n;
        origin.y /= n;
        origin.z /= n;
        return origin;
    }

    tf2::Vector3 corrected_point(
        const tf2::Quaternion& q_fix,
        const RawSample& sample,
        const Origin& origin) const {
        const tf2::Vector3 t_li(MID360_IMU2LIDAR_X, MID360_IMU2LIDAR_Y, MID360_IMU2LIDAR_Z);
        const tf2::Vector3 correction = tf2::quatRotate(sample.q, t_li);

        tf2::Vector3 p(
            sample.x - origin.x - correction.x(),
            sample.y - origin.y - correction.y(),
            sample.z - origin.z - correction.z());
        return tf2::quatRotate(q_fix, p);
    }

    void wait_for_motion_start(const RawSample& sample, const tf2::Vector3& seed_p) {
        if (!have_start_reference_) {
            start_ref_raw_ = sample;
            start_ref_x_ = seed_p.x();
            start_ref_y_ = seed_p.y();
            have_start_reference_ = true;
            RCLCPP_WARN(get_logger(), "Waiting for movement to start...");
            return;
        }

        const double motion = norm2(seed_p.x() - start_ref_x_, seed_p.y() - start_ref_y_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "WAIT_MOVE: motion=%.3f/%.3f m", motion, start_min_motion_m_);
        if (motion < start_min_motion_m_) {
            return;
        }

        capture_started_ = true;
        move_samples_.clear();
        seed_xy_history_.clear();
        move_samples_.push_back(start_ref_raw_);
        move_samples_.push_back(sample);
        seed_xy_history_.push_back({start_ref_x_, start_ref_y_});
        seed_xy_history_.push_back({seed_p.x(), seed_p.y()});
        RCLCPP_WARN(get_logger(), "Movement detected. Collecting flat-ground motion samples.");
    }

    double seed_motion_distance() const {
        if (seed_xy_history_.empty()) return 0.0;
        const auto& first = seed_xy_history_.front();
        const auto& last = seed_xy_history_.back();
        return norm2(last.first - first.first, last.second - first.second);
    }

    bool motion_stopped() const {
        if (static_cast<int>(seed_xy_history_.size()) <= stop_window_samples_) {
            return false;
        }
        const auto& last = seed_xy_history_.back();
        const auto& prev = seed_xy_history_[seed_xy_history_.size() - stop_window_samples_];
        return norm2(last.first - prev.first, last.second - prev.second) <= stop_motion_m_;
    }

    Metrics evaluate(double roll_deg, double pitch_deg) const {
        Metrics m;
        m.roll_deg = roll_deg;
        m.pitch_deg = pitch_deg;
        if (warmup_samples_.empty() || move_samples_.size() < 2) {
            return m;
        }

        tf2::Quaternion q_fix;
        q_fix.setRPY(deg_to_rad(roll_deg), deg_to_rad(pitch_deg), deg_to_rad(seed_fix_yaw_deg_));
        q_fix.normalize();

        const Origin origin = compute_origin(warmup_samples_);
        std::vector<double> z_values;
        std::vector<double> s_values;
        z_values.reserve(move_samples_.size());
        s_values.reserve(move_samples_.size());

        bool have_last = false;
        double last_x = 0.0;
        double last_y = 0.0;
        double path_s = 0.0;
        double first_z = 0.0;
        for (std::size_t i = 0; i < move_samples_.size(); ++i) {
            const tf2::Vector3 p = corrected_point(q_fix, move_samples_[i], origin);
            if (i == 0) {
                first_z = p.z();
            }
            if (have_last) {
                path_s += norm2(p.x() - last_x, p.y() - last_y);
            }
            have_last = true;
            last_x = p.x();
            last_y = p.y();
            z_values.push_back(p.z() - first_z);
            s_values.push_back(path_s);
        }

        m.xy_distance = path_s;
        if (m.xy_distance < std::max(0.1, move_min_distance_m_ * 0.5)) {
            return m;
        }

        double sum_z = 0.0;
        double sum_s = 0.0;
        for (std::size_t i = 0; i < z_values.size(); ++i) {
            sum_z += z_values[i];
            sum_s += s_values[i];
        }
        const double mean_z = sum_z / static_cast<double>(z_values.size());
        const double mean_s = sum_s / static_cast<double>(s_values.size());

        double sq = 0.0;
        double max_abs = 0.0;
        double min_z = z_values.front();
        double max_z = z_values.front();
        double cov_sz = 0.0;
        double var_s = 0.0;
        for (std::size_t i = 0; i < z_values.size(); ++i) {
            const double z_centered = z_values[i] - mean_z;
            sq += z_centered * z_centered;
            max_abs = std::max(max_abs, std::fabs(z_values[i]));
            min_z = std::min(min_z, z_values[i]);
            max_z = std::max(max_z, z_values[i]);
            cov_sz += (s_values[i] - mean_s) * (z_values[i] - mean_z);
            var_s += (s_values[i] - mean_s) * (s_values[i] - mean_s);
        }

        m.z_rms = std::sqrt(sq / static_cast<double>(z_values.size()));
        m.z_max_abs = max_abs;
        m.z_peak_to_peak = max_z - min_z;
        m.z_slope_abs = var_s > 1e-9 ? std::fabs(cov_sz / var_s) : 0.0;
        m.score = m.z_rms + 0.25 * m.z_peak_to_peak + 0.20 * m.z_slope_abs;
        m.valid = std::isfinite(m.score);
        return m;
    }

    Metrics optimize() const {
        Metrics best = evaluate(seed_fix_roll_deg_, seed_fix_pitch_deg_);
        if (!best.valid) {
            best.score = std::numeric_limits<double>::infinity();
        }

        double center_roll = seed_fix_roll_deg_;
        double center_pitch = seed_fix_pitch_deg_;
        double roll_range = std::fabs(search_roll_deg_);
        double pitch_range = std::fabs(search_pitch_deg_);
        double step = search_step_deg_;

        for (int round = 0; round < optimizer_rounds_; ++round) {
            const int roll_steps = static_cast<int>(std::ceil(roll_range / step));
            const int pitch_steps = static_cast<int>(std::ceil(pitch_range / step));
            for (int ir = -roll_steps; ir <= roll_steps; ++ir) {
                for (int ip = -pitch_steps; ip <= pitch_steps; ++ip) {
                    const Metrics candidate = evaluate(
                        center_roll + ir * step,
                        center_pitch + ip * step);
                    if (candidate.valid && candidate.score < best.score) {
                        best = candidate;
                    }
                }
            }
            center_roll = best.roll_deg;
            center_pitch = best.pitch_deg;
            roll_range = std::max(step, step * 2.0);
            pitch_range = std::max(step, step * 2.0);
            step *= 0.5;

            RCLCPP_INFO(get_logger(),
                "OPT round %d/%d best: roll=%.4f pitch=%.4f z_rms=%.4f z_max=%.4f z_p2p=%.4f slope=%.4f",
                round + 1, optimizer_rounds_, best.roll_deg, best.pitch_deg,
                best.z_rms, best.z_max_abs, best.z_peak_to_peak, best.z_slope_abs);
        }
        return best;
    }

    void print_result(const Metrics& best) const {
        if (!best.valid) {
            RCLCPP_ERROR(get_logger(),
                "Dynamic tilt calibration failed. Move farther on flat ground and try again.");
            return;
        }

        RCLCPP_INFO(get_logger(), "========== DYNAMIC TILT CALIBRATION RESULTS ==========");
        RCLCPP_INFO(get_logger(), "samples=%zu warmup=%zu xy_path=%.3f m",
            move_samples_.size(), warmup_samples_.size(), best.xy_distance);
        RCLCPP_INFO(get_logger(), "fix_roll_deg=%.6f", best.roll_deg);
        RCLCPP_INFO(get_logger(), "fix_pitch_deg=%.6f", best.pitch_deg);
        RCLCPP_INFO(get_logger(), "fix_yaw_deg=%.6f", seed_fix_yaw_deg_);
        RCLCPP_INFO(get_logger(),
            "quality: z_rms=%.6f m z_max_abs=%.6f m z_peak_to_peak=%.6f m z_slope=%.6f m/m score=%.6f",
            best.z_rms, best.z_max_abs, best.z_peak_to_peak, best.z_slope_abs, best.score);
        RCLCPP_INFO(get_logger(), "Recommended postion_odom.sh parameters:");
        RCLCPP_INFO(get_logger(), "  FIX_ROLL_DEG=\"%.6f\"", best.roll_deg);
        RCLCPP_INFO(get_logger(), "  FIX_PITCH_DEG=\"%.6f\"", best.pitch_deg);
        RCLCPP_INFO(get_logger(), "  FIX_YAW_DEG=\"%.6f\"", seed_fix_yaw_deg_);
        RCLCPP_INFO(get_logger(), "======================================================");
    }

    std::string source_frame_;
    std::string target_frame_;
    double sample_rate_hz_ = 80.0;
    int warmup_frames_ = 120;
    int move_min_samples_ = 200;
    int move_max_samples_ = 3000;
    double move_min_distance_m_ = 0.8;
    double start_min_motion_m_ = 0.03;
    int stop_window_samples_ = 80;
    double stop_motion_m_ = 0.02;

    double seed_fix_roll_deg_ = 0.0;
    double seed_fix_pitch_deg_ = -56.39;
    double seed_fix_yaw_deg_ = 0.0;
    double search_roll_deg_ = 6.0;
    double search_pitch_deg_ = 6.0;
    double search_step_deg_ = 1.0;
    int optimizer_rounds_ = 5;

    tf2::Quaternion seed_fix_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool have_last_tf_stamp_ = false;
    rclcpp::Time last_tf_stamp_;

    std::vector<RawSample> warmup_samples_;
    std::vector<RawSample> move_samples_;
    std::vector<std::pair<double, double>> seed_xy_history_;
    Origin seed_origin_;

    bool have_start_reference_ = false;
    bool capture_started_ = false;
    RawSample start_ref_raw_;
    double start_ref_x_ = 0.0;
    double start_ref_y_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TiltDynamicCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
