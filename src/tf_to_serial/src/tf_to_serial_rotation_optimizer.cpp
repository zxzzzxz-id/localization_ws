#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double MID360_IMU2LIDAR_X = 0.011;
constexpr double MID360_IMU2LIDAR_Y = 0.02329;
constexpr double MID360_IMU2LIDAR_Z = -0.04412;

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

class TfToSerialRotationOptimizer : public rclcpp::Node {
public:
    TfToSerialRotationOptimizer() : Node("tf_to_serial_rotation_optimizer") {
        source_frame_ = declare_parameter<std::string>("source_frame", "camera_init");
        target_frame_ = declare_parameter<std::string>("target_frame", "body_hf");
        sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 80.0);
        warmup_frames_ = declare_parameter<int>("warmup_frames", 120);
        rotate_min_samples_ = declare_parameter<int>("rotate_min_samples", 500);
        rotate_max_samples_ = declare_parameter<int>("rotate_max_samples", 3000);
        rotate_finish_yaw_span_deg_ = declare_parameter<double>("rotate_finish_yaw_span_deg", 330.0);
        rotate_stop_min_yaw_span_deg_ = declare_parameter<double>("rotate_stop_min_yaw_span_deg", 120.0);
        rotate_finish_stable_samples_ = declare_parameter<int>("rotate_finish_stable_samples", 120);
        rotate_finish_stable_yaw_deg_ = declare_parameter<double>("rotate_finish_stable_yaw_deg", 2.0);
        rotate_start_min_yaw_deg_ = declare_parameter<double>("rotate_start_min_yaw_deg", 3.0);
        rotate_start_min_motion_m_ = declare_parameter<double>("rotate_start_min_motion_m", 0.02);

        seed_fix_roll_deg_ = declare_parameter<double>("seed_fix_roll_deg", 0.0);
        seed_fix_pitch_deg_ = declare_parameter<double>("seed_fix_pitch_deg", -56.39);
        seed_fix_yaw_deg_ = declare_parameter<double>("seed_fix_yaw_deg", 0.0);
        search_roll_deg_ = declare_parameter<double>("search_roll_deg", 4.0);
        search_pitch_deg_ = declare_parameter<double>("search_pitch_deg", 4.0);
        search_yaw_deg_ = declare_parameter<double>("search_yaw_deg", 2.0);
        search_step_deg_ = declare_parameter<double>("search_step_deg", 1.0);
        optimizer_rounds_ = declare_parameter<int>("optimizer_rounds", 4);

        if (sample_rate_hz_ <= 0.0) sample_rate_hz_ = 80.0;
        if (warmup_frames_ < 10) warmup_frames_ = 10;
        if (rotate_min_samples_ < 50) rotate_min_samples_ = 50;
        if (rotate_max_samples_ < rotate_min_samples_) rotate_max_samples_ = rotate_min_samples_;
        if (rotate_finish_stable_samples_ < 5) rotate_finish_stable_samples_ = 5;
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
            std::bind(&TfToSerialRotationOptimizer::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Rotation optimizer started.");
        RCLCPP_INFO(get_logger(), "Keep still for warmup, then rotate in place around the robot center.");
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

    struct State {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct Candidate {
        double fix_roll_deg = 0.0;
        double fix_pitch_deg = 0.0;
        double fix_yaw_deg = 0.0;
        double lidar2robot_dis = 0.0;
        double lidar2robot_ang = 0.0;
        double robot_xy_rms = std::numeric_limits<double>::infinity();
        double robot_xy_max = std::numeric_limits<double>::infinity();
        double circle_rms = std::numeric_limits<double>::infinity();
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
                compute_seed_warmup_reference();
                RCLCPP_WARN(get_logger(), "Warmup done. Start slow in-place rotation now.");
            }
            return;
        }

        State state;
        if (!build_seed_state(sample, state)) {
            return;
        }

        if (!capture_started_) {
            wait_for_rotation_start(sample, state);
            return;
        }

        rotation_samples_.push_back(sample);
        update_seed_yaw_span(state.yaw);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "ROTATE_CAPTURE: fresh_tf_samples=%zu/%d yaw_span=%.1f deg finish=%.1f stop_min=%.1f stable_stop=%s",
            rotation_samples_.size(), rotate_max_samples_,
            rad_to_deg(seed_max_yaw_ - seed_min_yaw_), rotate_finish_yaw_span_deg_,
            rotate_stop_min_yaw_span_deg_,
            rotation_stopped() ? "yes" : "no");

        const bool enough_samples =
            static_cast<int>(rotation_samples_.size()) >= rotate_min_samples_;
        const double yaw_span_deg = rad_to_deg(seed_max_yaw_ - seed_min_yaw_);
        const bool enough_yaw =
            yaw_span_deg >= rotate_finish_yaw_span_deg_;
        const bool stopped_after_useful_rotation =
            yaw_span_deg >= rotate_stop_min_yaw_span_deg_ &&
            static_cast<int>(rotation_samples_.size()) >=
            rotate_min_samples_ + rotate_finish_stable_samples_ && rotation_stopped();
        const bool forced_stop =
            static_cast<int>(rotation_samples_.size()) >= rotate_max_samples_;

        if ((enough_samples && (enough_yaw || stopped_after_useful_rotation)) || forced_stop) {
            if (forced_stop) {
                RCLCPP_WARN(get_logger(), "Max samples reached; optimizing with collected rotation data.");
            } else if (stopped_after_useful_rotation) {
                RCLCPP_WARN(get_logger(), "Rotation stopped after useful yaw span; optimizing now.");
            }
            const Candidate best = optimize();
            print_result(best);
            rclcpp::shutdown();
        }
    }

    void compute_seed_warmup_reference() {
        seed_avg_x_ = 0.0;
        seed_avg_y_ = 0.0;
        seed_avg_z_ = 0.0;
        seed_yaw_sin_ = 0.0;
        seed_yaw_cos_ = 0.0;

        for (const auto& s : warmup_samples_) {
            seed_avg_x_ += s.x;
            seed_avg_y_ += s.y;
            seed_avg_z_ += s.z;
            const double yaw = leveled_yaw(seed_fix_, s.q);
            seed_yaw_sin_ += std::sin(yaw);
            seed_yaw_cos_ += std::cos(yaw);
        }
        const double n = static_cast<double>(warmup_samples_.size());
        seed_avg_x_ /= n;
        seed_avg_y_ /= n;
        seed_avg_z_ /= n;
        seed_avg_yaw_ = std::atan2(seed_yaw_sin_, seed_yaw_cos_);
    }

    double leveled_yaw(const tf2::Quaternion& q_fix, const tf2::Quaternion& q) const {
        tf2::Quaternion q_leveled = q_fix * q;
        q_leveled.normalize();
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q_leveled).getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool build_seed_state(const RawSample& sample, State& state) {
        state.x = sample.x - seed_avg_x_;
        state.y = sample.y - seed_avg_y_;
        const double yaw_wrapped = wrap_to_pi(leveled_yaw(seed_fix_, sample.q) - seed_avg_yaw_);

        if (!seed_have_last_yaw_) {
            seed_have_last_yaw_ = true;
            seed_last_yaw_wrapped_ = yaw_wrapped;
            seed_yaw_unwrapped_ = yaw_wrapped;
        } else {
            seed_yaw_unwrapped_ += wrap_to_pi(yaw_wrapped - seed_last_yaw_wrapped_);
            seed_last_yaw_wrapped_ = yaw_wrapped;
        }
        state.yaw = seed_yaw_unwrapped_;
        return true;
    }

    void wait_for_rotation_start(const RawSample& sample, const State& state) {
        if (!have_start_reference_) {
            start_ref_raw_ = sample;
            start_ref_x_ = state.x;
            start_ref_y_ = state.y;
            start_ref_yaw_ = state.yaw;
            have_start_reference_ = true;
            RCLCPP_WARN(get_logger(), "Waiting for rotation to start...");
            return;
        }

        const double motion = norm2(state.x - start_ref_x_, state.y - start_ref_y_);
        const double yaw_delta_deg = std::fabs(rad_to_deg(state.yaw - start_ref_yaw_));
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "WAIT_ROTATION: motion=%.3f/%.3f m yaw_delta=%.2f/%.2f deg",
            motion, rotate_start_min_motion_m_, yaw_delta_deg, rotate_start_min_yaw_deg_);

        if (motion < rotate_start_min_motion_m_ && yaw_delta_deg < rotate_start_min_yaw_deg_) {
            return;
        }

        capture_started_ = true;
        rotation_samples_.clear();
        rotation_samples_.push_back(start_ref_raw_);
        rotation_samples_.push_back(sample);
        seed_min_yaw_ = std::min(start_ref_yaw_, state.yaw);
        seed_max_yaw_ = std::max(start_ref_yaw_, state.yaw);
        RCLCPP_WARN(get_logger(), "Rotation detected. Collecting raw TF samples.");
    }

    void update_seed_yaw_span(double yaw) {
        seed_min_yaw_ = std::min(seed_min_yaw_, yaw);
        seed_max_yaw_ = std::max(seed_max_yaw_, yaw);
        seed_yaw_history_.push_back(yaw);
    }

    bool rotation_stopped() const {
        if (static_cast<int>(seed_yaw_history_.size()) <= rotate_finish_stable_samples_) {
            return false;
        }

        const double recent_span = std::fabs(rad_to_deg(
            seed_yaw_history_.back() -
            seed_yaw_history_[seed_yaw_history_.size() - rotate_finish_stable_samples_]));
        return recent_span <= rotate_finish_stable_yaw_deg_;
    }

    bool build_states(
        const tf2::Quaternion& q_fix,
        const std::vector<RawSample>& samples,
        std::vector<State>& states) const {
        if (warmup_samples_.empty() || samples.size() < 20) {
            return false;
        }

        double avg_x = 0.0;
        double avg_y = 0.0;
        double avg_z = 0.0;
        double yaw_sin = 0.0;
        double yaw_cos = 0.0;
        for (const auto& s : warmup_samples_) {
            avg_x += s.x;
            avg_y += s.y;
            avg_z += s.z;
            const double yaw = leveled_yaw(q_fix, s.q);
            yaw_sin += std::sin(yaw);
            yaw_cos += std::cos(yaw);
        }
        const double n = static_cast<double>(warmup_samples_.size());
        avg_x /= n;
        avg_y /= n;
        avg_z /= n;
        const double avg_yaw = std::atan2(yaw_sin, yaw_cos);

        const tf2::Vector3 t_li(MID360_IMU2LIDAR_X, MID360_IMU2LIDAR_Y, MID360_IMU2LIDAR_Z);
        states.clear();
        states.reserve(samples.size());

        bool have_last_yaw = false;
        double last_yaw_wrapped = 0.0;
        double yaw_unwrapped = 0.0;

        for (const auto& s : samples) {
            const double x = s.x - avg_x;
            const double y = s.y - avg_y;
            const double z = s.z - avg_z;
            const tf2::Vector3 correction = tf2::quatRotate(s.q, t_li);
            tf2::Vector3 p_vec(
                x - correction.x(),
                y - correction.y(),
                z - correction.z());
            p_vec = tf2::quatRotate(q_fix, p_vec);

            const double yaw_wrapped = wrap_to_pi(leveled_yaw(q_fix, s.q) - avg_yaw);
            if (!have_last_yaw) {
                have_last_yaw = true;
                last_yaw_wrapped = yaw_wrapped;
                yaw_unwrapped = yaw_wrapped;
            } else {
                yaw_unwrapped += wrap_to_pi(yaw_wrapped - last_yaw_wrapped);
                last_yaw_wrapped = yaw_wrapped;
            }

            states.push_back({p_vec.x(), p_vec.y(), yaw_unwrapped});
        }
        return true;
    }

    bool fit_circle(const std::vector<State>& states, double& cx, double& cy, double& radius) const {
        if (states.size() < 20) return false;

        double sx = 0.0, sy = 0.0, sx2 = 0.0, sy2 = 0.0, sxy = 0.0;
        double sr2 = 0.0, sxr2 = 0.0, syr2 = 0.0;
        for (const auto& s : states) {
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

        const double n = static_cast<double>(states.size());
        std::array<std::array<double, 4>, 3> a = {{
            {{sx2, sxy, sx, -sxr2}},
            {{sxy, sy2, sy, -syr2}},
            {{sx,  sy,  n,  -sr2}},
        }};

        std::array<double, 3> abc{};
        if (!solve_3x3(a, abc)) return false;

        const double radius_sq = 0.25 * (abc[0] * abc[0] + abc[1] * abc[1]) - abc[2];
        if (radius_sq <= 0.0 || !std::isfinite(radius_sq)) return false;

        cx = -0.5 * abc[0];
        cy = -0.5 * abc[1];
        radius = std::sqrt(radius_sq);
        return true;
    }

    Candidate evaluate(double roll_deg, double pitch_deg, double yaw_deg) const {
        Candidate c;
        c.fix_roll_deg = roll_deg;
        c.fix_pitch_deg = pitch_deg;
        c.fix_yaw_deg = yaw_deg;

        tf2::Quaternion q_fix;
        q_fix.setRPY(deg_to_rad(roll_deg), deg_to_rad(pitch_deg), deg_to_rad(yaw_deg));
        q_fix.normalize();

        std::vector<State> states;
        if (!build_states(q_fix, rotation_samples_, states)) {
            return c;
        }

        double cx = 0.0;
        double cy = 0.0;
        double radius = 0.0;
        if (!fit_circle(states, cx, cy, radius)) {
            return c;
        }

        double phase_sin = 0.0;
        double phase_cos = 0.0;
        double circle_sq = 0.0;
        for (const auto& s : states) {
            const double r = norm2(s.x - cx, s.y - cy);
            circle_sq += (r - radius) * (r - radius);
            const double phase = wrap_to_pi(std::atan2(s.y - cy, s.x - cx) - s.yaw);
            phase_sin += std::sin(phase);
            phase_cos += std::cos(phase);
        }
        const double phase = std::atan2(phase_sin, phase_cos);

        std::vector<std::pair<double, double>> robot_xy;
        robot_xy.reserve(states.size());
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const auto& s : states) {
            const double rx = s.x - radius * std::cos(phase + s.yaw);
            const double ry = s.y - radius * std::sin(phase + s.yaw);
            robot_xy.emplace_back(rx, ry);
            mean_x += rx;
            mean_y += ry;
        }
        mean_x /= static_cast<double>(robot_xy.size());
        mean_y /= static_cast<double>(robot_xy.size());

        double residual_sq = 0.0;
        double residual_max = 0.0;
        for (const auto& p : robot_xy) {
            const double e = norm2(p.first - mean_x, p.second - mean_y);
            residual_sq += e * e;
            residual_max = std::max(residual_max, e);
        }

        c.lidar2robot_dis = radius;
        c.lidar2robot_ang = wrap_to_180(rad_to_deg(phase));
        c.circle_rms = std::sqrt(circle_sq / static_cast<double>(states.size()));
        c.robot_xy_rms = std::sqrt(residual_sq / static_cast<double>(robot_xy.size()));
        c.robot_xy_max = residual_max;
        c.score = c.robot_xy_rms + 0.25 * c.circle_rms;
        c.valid = std::isfinite(c.score);
        return c;
    }

    Candidate optimize() const {
        Candidate best = evaluate(seed_fix_roll_deg_, seed_fix_pitch_deg_, seed_fix_yaw_deg_);
        if (!best.valid) {
            RCLCPP_WARN(get_logger(), "Seed evaluation failed; grid search will still run.");
            best.score = std::numeric_limits<double>::infinity();
        }

        double center_roll = seed_fix_roll_deg_;
        double center_pitch = seed_fix_pitch_deg_;
        double center_yaw = seed_fix_yaw_deg_;
        double roll_range = std::fabs(search_roll_deg_);
        double pitch_range = std::fabs(search_pitch_deg_);
        double yaw_range = std::fabs(search_yaw_deg_);
        double step = search_step_deg_;

        for (int round = 0; round < optimizer_rounds_; ++round) {
            const int roll_steps = static_cast<int>(std::ceil(roll_range / step));
            const int pitch_steps = static_cast<int>(std::ceil(pitch_range / step));
            const int yaw_steps = static_cast<int>(std::ceil(yaw_range / step));

            for (int ir = -roll_steps; ir <= roll_steps; ++ir) {
                for (int ip = -pitch_steps; ip <= pitch_steps; ++ip) {
                    for (int iy = -yaw_steps; iy <= yaw_steps; ++iy) {
                        const double roll = center_roll + ir * step;
                        const double pitch = center_pitch + ip * step;
                        const double yaw = center_yaw + iy * step;
                        const Candidate c = evaluate(roll, pitch, yaw);
                        if (c.valid && c.score < best.score) {
                            best = c;
                        }
                    }
                }
            }

            center_roll = best.fix_roll_deg;
            center_pitch = best.fix_pitch_deg;
            center_yaw = best.fix_yaw_deg;
            roll_range = std::max(step, step * 2.0);
            pitch_range = std::max(step, step * 2.0);
            yaw_range = std::max(step, step * 2.0);
            step *= 0.5;

            RCLCPP_INFO(get_logger(),
                "OPT round %d/%d best: fix=(%.4f %.4f %.4f) dis=%.5f ang=%.4f xy_rms=%.4f xy_max=%.4f",
                round + 1, optimizer_rounds_, best.fix_roll_deg, best.fix_pitch_deg,
                best.fix_yaw_deg, best.lidar2robot_dis, best.lidar2robot_ang,
                best.robot_xy_rms, best.robot_xy_max);
        }

        return best;
    }

    void print_result(const Candidate& best) const {
        if (!best.valid) {
            RCLCPP_ERROR(get_logger(), "Rotation optimization failed. Check TF and rotate in place again.");
            return;
        }
        RCLCPP_INFO(get_logger(), "========== TF_TO_SERIAL ROTATION OPTIMIZATION RESULTS ==========");
        RCLCPP_INFO(get_logger(), "samples=%zu warmup=%zu", rotation_samples_.size(), warmup_samples_.size());
        RCLCPP_INFO(get_logger(), "fix_roll_deg=%.6f", best.fix_roll_deg);
        RCLCPP_INFO(get_logger(), "fix_pitch_deg=%.6f", best.fix_pitch_deg);
        RCLCPP_INFO(get_logger(), "fix_yaw_deg=%.6f", best.fix_yaw_deg);
        RCLCPP_INFO(get_logger(), "lidar2robot_dis=%.6f m", best.lidar2robot_dis);
        RCLCPP_INFO(get_logger(), "lidar2robot_ang=%.6f deg", best.lidar2robot_ang);
        RCLCPP_INFO(get_logger(),
            "quality: robot_xy_rms=%.6f m robot_xy_max=%.6f m circle_rms=%.6f m score=%.6f",
            best.robot_xy_rms, best.robot_xy_max, best.circle_rms, best.score);
        RCLCPP_INFO(get_logger(), "Recommended postion_odom.sh parameters:");
        RCLCPP_INFO(get_logger(), "  FIX_ROLL_DEG=\"%.6f\"", best.fix_roll_deg);
        RCLCPP_INFO(get_logger(), "  FIX_PITCH_DEG=\"%.6f\"", best.fix_pitch_deg);
        RCLCPP_INFO(get_logger(), "  FIX_YAW_DEG=\"%.6f\"", best.fix_yaw_deg);
        RCLCPP_INFO(get_logger(), "  LIDAR2ROBOT_DIS=\"%.6f\"", best.lidar2robot_dis);
        RCLCPP_INFO(get_logger(), "  LIDAR2ROBOT_ANG=\"%.6f\"", best.lidar2robot_ang);
        RCLCPP_INFO(get_logger(), "================================================================");
    }

    std::string source_frame_;
    std::string target_frame_;
    double sample_rate_hz_ = 80.0;
    int warmup_frames_ = 120;
    int rotate_min_samples_ = 500;
    int rotate_max_samples_ = 3000;
    double rotate_finish_yaw_span_deg_ = 330.0;
    double rotate_stop_min_yaw_span_deg_ = 120.0;
    int rotate_finish_stable_samples_ = 120;
    double rotate_finish_stable_yaw_deg_ = 2.0;
    double rotate_start_min_yaw_deg_ = 3.0;
    double rotate_start_min_motion_m_ = 0.02;

    double seed_fix_roll_deg_ = 0.0;
    double seed_fix_pitch_deg_ = -56.39;
    double seed_fix_yaw_deg_ = 0.0;
    double search_roll_deg_ = 4.0;
    double search_pitch_deg_ = 4.0;
    double search_yaw_deg_ = 2.0;
    double search_step_deg_ = 1.0;
    int optimizer_rounds_ = 4;

    tf2::Quaternion seed_fix_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool have_last_tf_stamp_ = false;
    rclcpp::Time last_tf_stamp_;

    std::vector<RawSample> warmup_samples_;
    std::vector<RawSample> rotation_samples_;

    double seed_avg_x_ = 0.0;
    double seed_avg_y_ = 0.0;
    double seed_avg_z_ = 0.0;
    double seed_yaw_sin_ = 0.0;
    double seed_yaw_cos_ = 0.0;
    double seed_avg_yaw_ = 0.0;
    bool seed_have_last_yaw_ = false;
    double seed_last_yaw_wrapped_ = 0.0;
    double seed_yaw_unwrapped_ = 0.0;
    double seed_min_yaw_ = 0.0;
    double seed_max_yaw_ = 0.0;
    std::vector<double> seed_yaw_history_;

    bool have_start_reference_ = false;
    bool capture_started_ = false;
    RawSample start_ref_raw_;
    double start_ref_x_ = 0.0;
    double start_ref_y_ = 0.0;
    double start_ref_yaw_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfToSerialRotationOptimizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
