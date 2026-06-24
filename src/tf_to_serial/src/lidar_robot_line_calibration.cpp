#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
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

}  // namespace

class LidarRobotLineCalibration : public rclcpp::Node {
public:
    LidarRobotLineCalibration() : Node("lidar_robot_line_calibration") {
        source_frame_ = declare_parameter<std::string>("source_frame", "camera_init");
        target_frame_ = declare_parameter<std::string>("target_frame", "body_hf");
        sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 50.0);
        warmup_frames_ = declare_parameter<int>("warmup_frames", 80);
        line_min_samples_ = declare_parameter<int>("line_min_samples", 150);
        line_max_samples_ = declare_parameter<int>("line_max_samples", 2500);
        line_min_distance_m_ = declare_parameter<double>("line_min_distance_m", 0.35);
        line_max_yaw_change_deg_ = declare_parameter<double>("line_max_yaw_change_deg", 8.0);
        line_lateral_ratio_ok_ = declare_parameter<double>("line_lateral_ratio_ok", 0.25);
        line_max_lateral_rms_m_ = declare_parameter<double>("line_max_lateral_rms_m", 0.04);
        line_max_lateral_abs_m_ = declare_parameter<double>("line_max_lateral_abs_m", 0.10);
        line_stable_angle_deg_ = declare_parameter<double>("line_stable_angle_deg", 1.0);
        line_min_stable_windows_ = declare_parameter<int>("line_min_stable_windows", 3);
        line_complete_on_stop_ = declare_parameter<bool>("line_complete_on_stop", true);
        line_stop_window_samples_ = declare_parameter<int>("line_stop_window_samples", 20);
        line_stop_motion_m_ = declare_parameter<double>("line_stop_motion_m", 0.015);
        observable_phase_deg_ = declare_parameter<double>("observable_phase_deg", 999.0);
        gravity_fix_roll_deg_ = declare_parameter<double>("gravity_fix_roll_deg", 0.0);
        gravity_fix_pitch_deg_ =
            declare_parameter<double>("gravity_fix_pitch_deg", -(180.0 - 55.1425));
        gravity_fix_yaw_deg_ = declare_parameter<double>("gravity_fix_yaw_deg", 0.0);

        if (sample_rate_hz_ <= 0.0) sample_rate_hz_ = 50.0;
        if (warmup_frames_ < 1) warmup_frames_ = 1;
        if (line_min_samples_ < 20) line_min_samples_ = 20;
        if (line_max_samples_ < line_min_samples_) line_max_samples_ = line_min_samples_;
        if (line_stop_window_samples_ < 2) line_stop_window_samples_ = 2;

        q_fix_.setRPY(
            deg_to_rad(gravity_fix_roll_deg_),
            deg_to_rad(gravity_fix_pitch_deg_),
            deg_to_rad(gravity_fix_yaw_deg_));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto period = std::chrono::duration<double>(1.0 / sample_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&LidarRobotLineCalibration::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Line calibration started.");
        RCLCPP_INFO(get_logger(), "Keep still for warmup. Then follow prompts: +X, -X, +Y, -Y.");
    }

private:
    enum class Stage {
        Warmup,
        SettlePosX,
        LinePosX,
        SettleNegX,
        LineNegX,
        SettlePosY,
        LinePosY,
        SettleNegY,
        LineNegY,
        Done
    };

    enum class Axis {
        PosX,
        NegX,
        PosY,
        NegY
    };

    struct State {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct Sample {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct LineResult {
        Axis axis = Axis::PosX;
        double distance = 0.0;
        double heading = 0.0;
        double yaw_span_deg = 0.0;
        double lateral_rms = 0.0;
        double lateral_max = 0.0;
        double recent_motion = std::numeric_limits<double>::infinity();
        bool accepted = false;
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
        State state;
        if (!build_state(tf, state)) {
            return;
        }

        switch (stage_) {
            case Stage::Warmup:
                break;
            case Stage::SettlePosX:
                handle_settle(state, Stage::LinePosX, "Move straight in robot +X direction now.");
                break;
            case Stage::LinePosX:
                handle_line(state, Axis::PosX, Stage::SettleNegX);
                break;
            case Stage::SettleNegX:
                handle_settle(state, Stage::LineNegX, "Move straight in robot -X direction now.");
                break;
            case Stage::LineNegX:
                handle_line(state, Axis::NegX, Stage::SettlePosY);
                break;
            case Stage::SettlePosY:
                handle_settle(state, Stage::LinePosY, "Move straight in robot +Y direction now.");
                break;
            case Stage::LinePosY:
                handle_line(state, Axis::PosY, Stage::SettleNegY);
                break;
            case Stage::SettleNegY:
                handle_settle(state, Stage::LineNegY, "Move straight in robot -Y direction now.");
                break;
            case Stage::LineNegY:
                handle_line(state, Axis::NegY, Stage::Done);
                break;
            case Stage::Done:
                break;
        }
    }

    bool build_state(const geometry_msgs::msg::TransformStamped& tf, State& state) {
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
            stage_ = Stage::SettlePosX;
            RCLCPP_WARN(get_logger(), "Warmup done. Keep still before +X segment.");
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

        state.x = p_vec.x();
        state.y = p_vec.y();
        state.yaw = unwrap_yaw(yaw_leveled - avg_yaw_leveled_);
        return true;
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

    void handle_settle(const State& state, Stage next_stage, const char* instruction) {
        settle_sum_x_ += state.x;
        settle_sum_y_ += state.y;
        settle_sum_yaw_ += state.yaw;
        ++settle_count_;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "SETTLE: keep still... %d/%d", settle_count_, warmup_frames_);

        if (settle_count_ < warmup_frames_) {
            return;
        }

        line_origin_x_ = settle_sum_x_ / settle_count_;
        line_origin_y_ = settle_sum_y_ / settle_count_;
        line_origin_yaw_ = settle_sum_yaw_ / settle_count_;
        line_samples_.clear();
        line_stable_windows_ = 0;
        have_last_line_angle_ = false;
        settle_count_ = 0;
        settle_sum_x_ = 0.0;
        settle_sum_y_ = 0.0;
        settle_sum_yaw_ = 0.0;
        stage_ = next_stage;
        RCLCPP_WARN(get_logger(), "%s", instruction);
    }

    void handle_line(const State& state, Axis axis, Stage next_stage) {
        line_samples_.push_back({state.x, state.y, state.yaw});
        auto result = evaluate_line(axis);
        publish_feedback(result);

        const bool enough =
            result.distance >= line_min_distance_m_ &&
            static_cast<int>(line_samples_.size()) >= line_min_samples_;
        const bool yaw_ok = result.yaw_span_deg <= line_max_yaw_change_deg_;
        const bool stable_ok = line_stable_windows_ >= line_min_stable_windows_;
        const bool straight_ok =
            result.lateral_rms <= line_max_lateral_rms_m_ &&
            result.lateral_max <= line_max_lateral_abs_m_;
        const bool stop_ok =
            !line_complete_on_stop_ || result.recent_motion <= line_stop_motion_m_;
        const bool direction_ok = result.accepted || results_.empty();
        const bool forced_stop = static_cast<int>(line_samples_.size()) >= line_max_samples_;

        if ((enough && yaw_ok && stable_ok && straight_ok && stop_ok && direction_ok) || forced_stop) {
            if (results_.empty() && enough && yaw_ok && stable_ok && straight_ok && stop_ok) {
                result.accepted = true;
            }
            results_.push_back(result);
            RCLCPP_WARN(get_logger(),
                "%s COMPLETE: distance=%.3f m heading=%.3f deg yaw_span=%.3f deg "
                "lateral_rms=%.3f m lateral_max=%.3f m accepted=%s",
                axis_name(axis), result.distance, wrap_to_180(rad_to_deg(result.heading)),
                result.yaw_span_deg, result.lateral_rms, result.lateral_max,
                result.accepted ? "true" : "false");

            if (next_stage == Stage::Done) {
                stage_ = Stage::Done;
                print_result();
                rclcpp::shutdown();
                return;
            }

            stage_ = next_stage;
            RCLCPP_WARN(get_logger(), "Keep still before next segment.");
        }
    }

    LineResult evaluate_line(Axis axis) {
        LineResult result;
        result.axis = axis;
        if (line_samples_.empty()) return result;

        const auto& last = line_samples_.back();
        const double dx = last.x - line_origin_x_;
        const double dy = last.y - line_origin_y_;
        result.distance = norm2(dx, dy);

        result.heading = fit_line_heading(dx, dy);

        double min_yaw = line_origin_yaw_;
        double max_yaw = line_origin_yaw_;
        for (const auto& s : line_samples_) {
            min_yaw = std::min(min_yaw, s.yaw);
            max_yaw = std::max(max_yaw, s.yaw);
        }
        result.yaw_span_deg = rad_to_deg(max_yaw - min_yaw);
        compute_lateral_error(result.heading, result.lateral_rms, result.lateral_max);
        result.recent_motion = recent_motion();

        if (have_last_line_angle_) {
            const double angle_delta = std::fabs(rad_to_deg(wrap_to_pi(result.heading - last_line_angle_)));
            if (angle_delta <= line_stable_angle_deg_) {
                ++line_stable_windows_;
            } else {
                line_stable_windows_ = 0;
            }
        }
        last_line_angle_ = result.heading;
        have_last_line_angle_ = true;

        const double expected = expected_heading(axis);
        const double err = wrap_to_pi(result.heading - expected);
        const double along = result.distance * std::cos(err);
        const double lateral = result.distance * std::sin(err);
        const double lateral_ratio =
            result.distance > 1e-6 ? std::fabs(lateral) / result.distance : 1.0;
        result.accepted = along > 0.0 && lateral_ratio <= line_lateral_ratio_ok_;
        return result;
    }

    double fit_line_heading(double fallback_dx, double fallback_dy) const {
        if (line_samples_.size() < 2) {
            return std::atan2(fallback_dy, fallback_dx);
        }

        double mean_x = line_origin_x_;
        double mean_y = line_origin_y_;
        for (const auto& s : line_samples_) {
            mean_x += s.x;
            mean_y += s.y;
        }
        const double n = static_cast<double>(line_samples_.size() + 1);
        mean_x /= n;
        mean_y /= n;

        double sxx = (line_origin_x_ - mean_x) * (line_origin_x_ - mean_x);
        double syy = (line_origin_y_ - mean_y) * (line_origin_y_ - mean_y);
        double sxy = (line_origin_x_ - mean_x) * (line_origin_y_ - mean_y);
        for (const auto& s : line_samples_) {
            const double x = s.x - mean_x;
            const double y = s.y - mean_y;
            sxx += x * x;
            syy += y * y;
            sxy += x * y;
        }

        double heading = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
        if (!std::isfinite(heading)) {
            heading = std::atan2(fallback_dy, fallback_dx);
        }

        if (std::cos(heading) * fallback_dx + std::sin(heading) * fallback_dy < 0.0) {
            heading = wrap_to_pi(heading + M_PI);
        }
        return heading;
    }

    void compute_lateral_error(double heading, double& rms, double& max_abs) const {
        if (line_samples_.empty()) {
            rms = 0.0;
            max_abs = 0.0;
            return;
        }

        double mean_x = line_origin_x_;
        double mean_y = line_origin_y_;
        for (const auto& s : line_samples_) {
            mean_x += s.x;
            mean_y += s.y;
        }
        const double n = static_cast<double>(line_samples_.size() + 1);
        mean_x /= n;
        mean_y /= n;

        const double nx = -std::sin(heading);
        const double ny = std::cos(heading);
        double sq = 0.0;
        max_abs = 0.0;
        for (const auto& s : line_samples_) {
            const double e = nx * (s.x - mean_x) + ny * (s.y - mean_y);
            sq += e * e;
            max_abs = std::max(max_abs, std::fabs(e));
        }
        rms = std::sqrt(sq / static_cast<double>(line_samples_.size()));
    }

    double recent_motion() const {
        if (static_cast<int>(line_samples_.size()) <= line_stop_window_samples_) {
            return std::numeric_limits<double>::infinity();
        }
        const auto& last = line_samples_.back();
        const auto& prev = line_samples_[line_samples_.size() - line_stop_window_samples_];
        return norm2(last.x - prev.x, last.y - prev.y);
    }

    double expected_heading(Axis axis) const {
        const double xy = xy_estimate_or_default();
        switch (axis) {
            case Axis::PosX: return xy;
            case Axis::NegX: return wrap_to_pi(xy + M_PI);
            case Axis::PosY: return wrap_to_pi(xy + M_PI_2);
            case Axis::NegY: return wrap_to_pi(xy - M_PI_2);
        }
        return xy;
    }

    double xy_estimate_or_default() const {
        if (!results_.empty()) {
            return estimate_xy_rotation(results_);
        }
        return deg_to_rad(180.0);
    }

    void publish_feedback(const LineResult& result) {
        const double expected = expected_heading(result.axis);
        const double err = wrap_to_pi(result.heading - expected);
        const double along = result.distance * std::cos(err);
        const double lateral = result.distance * std::sin(err);

        const char* status = "OK";
        if (results_.empty()) {
            if (result.distance < line_min_distance_m_) {
                status = "LEARNING_CONTINUE";
            } else if (result.lateral_rms > line_max_lateral_rms_m_ ||
                       result.lateral_max > line_max_lateral_abs_m_) {
                status = "NOT_STRAIGHT";
            } else if (line_complete_on_stop_ && result.recent_motion > line_stop_motion_m_) {
                status = "STOP_TO_COMPLETE";
            } else {
                status = "LEARNING_OK";
            }
        } else if (along < -0.05) {
            status = "REVERSED";
        } else if (std::fabs(lateral) > std::max(0.05, line_lateral_ratio_ok_ * result.distance)) {
            status = lateral > 0.0 ? "TOO_LEFT" : "TOO_RIGHT";
        } else if (result.distance < line_min_distance_m_) {
            status = "CONTINUE";
        } else if (result.lateral_rms > line_max_lateral_rms_m_ ||
                   result.lateral_max > line_max_lateral_abs_m_) {
            status = "NOT_STRAIGHT";
        } else if (line_complete_on_stop_ && result.recent_motion > line_stop_motion_m_) {
            status = "STOP_TO_COMPLETE";
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 600,
            "%s: %s dist=%.3f/%.3f m along=%.3f lateral=%.3f heading_err=%.2f deg "
            "yaw_span=%.2f/%.2f lateral_rms=%.3f/%.3f recent=%.3f/%.3f stable=%d/%d",
            axis_name(result.axis), status, result.distance, line_min_distance_m_,
            along, lateral, rad_to_deg(err), result.yaw_span_deg, line_max_yaw_change_deg_,
            result.lateral_rms, line_max_lateral_rms_m_, result.recent_motion, line_stop_motion_m_,
            line_stable_windows_, line_min_stable_windows_);
    }

    double estimate_xy_rotation(const std::vector<LineResult>& results) const {
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (const auto& r : results) {
            if (!r.accepted) continue;
            if (r.distance < line_min_distance_m_ * 0.5) continue;

            double axis_offset = 0.0;
            switch (r.axis) {
                case Axis::PosX: axis_offset = 0.0; break;
                case Axis::NegX: axis_offset = M_PI; break;
                case Axis::PosY: axis_offset = M_PI_2; break;
                case Axis::NegY: axis_offset = -M_PI_2; break;
            }

            const double xy = wrap_to_pi(r.heading - axis_offset);
            sin_sum += r.distance * std::sin(xy);
            cos_sum += r.distance * std::cos(xy);
        }

        if (std::fabs(sin_sum) < 1e-12 && std::fabs(cos_sum) < 1e-12) {
            return deg_to_rad(180.0);
        }
        return std::atan2(sin_sum, cos_sum);
    }

    void print_result() const {
        const double xy_rotation = estimate_xy_rotation(results_);
        double line_error_sq = 0.0;
        int line_error_count = 0;

        for (const auto& r : results_) {
            if (!r.accepted) continue;
            double axis_offset = 0.0;
            switch (r.axis) {
                case Axis::PosX: axis_offset = 0.0; break;
                case Axis::NegX: axis_offset = M_PI; break;
                case Axis::PosY: axis_offset = M_PI_2; break;
                case Axis::NegY: axis_offset = -M_PI_2; break;
            }
            const double expected = wrap_to_pi(xy_rotation + axis_offset);
            const double err = wrap_to_pi(r.heading - expected);
            line_error_sq += err * err;
            ++line_error_count;
        }

        const double line_rms_deg =
            line_error_count > 0 ? rad_to_deg(std::sqrt(line_error_sq / line_error_count)) : 0.0;

        RCLCPP_INFO(get_logger(), "========== LINE CALIBRATION RESULTS ==========");
        RCLCPP_INFO(get_logger(), "xy_rotation_deg=%.6f", wrap_to_180(rad_to_deg(xy_rotation)));
        RCLCPP_INFO(get_logger(), "quality: line_angle_rms=%.6f deg", line_rms_deg);
        if (std::fabs(observable_phase_deg_) <= 360.0) {
            RCLCPP_INFO(get_logger(), "lidar2robot_ang=%.6f deg", wrap_to_180(observable_phase_deg_));
        } else {
            RCLCPP_INFO(get_logger(),
                "observable_phase_deg was not provided; use the rotation calibration result as lidar2robot_ang.");
        }
        for (const auto& r : results_) {
            RCLCPP_INFO(get_logger(),
                "%s: distance=%.3f m heading=%.3f deg yaw_span=%.3f deg "
                "lateral_rms=%.3f m lateral_max=%.3f m accepted=%s",
                axis_name(r.axis), r.distance, wrap_to_180(rad_to_deg(r.heading)),
                r.yaw_span_deg, r.lateral_rms, r.lateral_max, r.accepted ? "true" : "false");
        }
        RCLCPP_INFO(get_logger(), "Recommended code parameter:");
        RCLCPP_INFO(get_logger(), "  xy_rotation_deg_ = %.6f;", wrap_to_180(rad_to_deg(xy_rotation)));
        if (std::fabs(observable_phase_deg_) <= 360.0) {
            RCLCPP_INFO(get_logger(), "  lidar2robot_ang_ = %.6f;", wrap_to_180(observable_phase_deg_));
        }
        RCLCPP_INFO(get_logger(), "==============================================");
    }

    const char* axis_name(Axis axis) const {
        switch (axis) {
            case Axis::PosX: return "+X";
            case Axis::NegX: return "-X";
            case Axis::PosY: return "+Y";
            case Axis::NegY: return "-Y";
        }
        return "?";
    }

    std::string source_frame_;
    std::string target_frame_;
    double sample_rate_hz_ = 50.0;
    int warmup_frames_ = 80;
    int line_min_samples_ = 150;
    int line_max_samples_ = 2500;
    double line_min_distance_m_ = 0.35;
    double line_max_yaw_change_deg_ = 8.0;
    double line_lateral_ratio_ok_ = 0.25;
    double line_max_lateral_rms_m_ = 0.04;
    double line_max_lateral_abs_m_ = 0.10;
    double line_stable_angle_deg_ = 1.0;
    int line_min_stable_windows_ = 3;
    bool line_complete_on_stop_ = true;
    int line_stop_window_samples_ = 20;
    double line_stop_motion_m_ = 0.015;
    double observable_phase_deg_ = 999.0;
    double gravity_fix_roll_deg_ = 0.0;
    double gravity_fix_pitch_deg_ = 0.0;
    double gravity_fix_yaw_deg_ = 0.0;

    tf2::Quaternion q_fix_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    Stage stage_ = Stage::Warmup;

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

    int settle_count_ = 0;
    double settle_sum_x_ = 0.0;
    double settle_sum_y_ = 0.0;
    double settle_sum_yaw_ = 0.0;
    double line_origin_x_ = 0.0;
    double line_origin_y_ = 0.0;
    double line_origin_yaw_ = 0.0;
    std::vector<Sample> line_samples_;
    std::vector<LineResult> results_;
    bool have_last_line_angle_ = false;
    double last_line_angle_ = 0.0;
    int line_stable_windows_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarRobotLineCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
