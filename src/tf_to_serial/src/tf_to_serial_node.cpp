#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <serial/serial.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <chrono>
#include <cstddef>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double MID360_IMU2LIDAR_X = 0.011;
constexpr double MID360_IMU2LIDAR_Y = 0.02329;
constexpr double MID360_IMU2LIDAR_Z = -0.04412;
constexpr double LIDAR2ROBOT_Z = 0.50;
constexpr int WARMUP_FRAMES = 20;
constexpr std::size_t kFrameFloatCount = 5;
constexpr std::size_t kFrameSize = 3 + kFrameFloatCount * sizeof(float) + 2;
constexpr uint8_t kFrameHeader0 = 0xFF;
constexpr uint8_t kFrameHeader1 = 0xFE;
constexpr uint8_t kFrameHeader2 = 0x01;
constexpr uint8_t kFrameTail0 = 0xAA;
constexpr uint8_t kFrameTail1 = 0xDD;

double wrapTo180(double ang_deg) {
    ang_deg = std::fmod(ang_deg + 180.0, 360.0);
    if (ang_deg < 0.0) {
        ang_deg += 360.0;
    }
    return ang_deg - 180.0;
}

double shortestAngleDeltaDeg(double current_deg, double last_deg) {
    return wrapTo180(current_deg - last_deg);
}

void appendFloat(std::vector<uint8_t>& frame, float value) {
    uint8_t bytes[sizeof(float)];
    std::memcpy(bytes, &value, sizeof(float));
    for (std::size_t i = 0; i < sizeof(float); ++i) {
        frame.push_back(bytes[i]);
    }
}

double computeSendValue(
    double current_value,
    double anchor_value,
    double target_value,
    double base_offset,
    bool anchor_valid) {
    if (!anchor_valid) {
        return current_value + base_offset;
    }
    return current_value - anchor_value + target_value + base_offset;
}

double computeSendYawDeg(
    double current_yaw_deg,
    double anchor_yaw_deg,
    double target_yaw_deg,
    double base_offset_yaw_deg,
    bool anchor_valid) {
    if (!anchor_valid) {
        return wrapTo180(current_yaw_deg + base_offset_yaw_deg);
    }
    const double delta_yaw_deg = shortestAngleDeltaDeg(current_yaw_deg, anchor_yaw_deg);
    return wrapTo180(target_yaw_deg + delta_yaw_deg + base_offset_yaw_deg);
}

}  // namespace

class TfToSerialNode : public rclcpp::Node {
public:
    TfToSerialNode()
        : Node("tf_to_serial_fastlio") {
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/lidar");
        baudrate_ = declare_parameter<int>("baudrate", 115200);
        world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
        body_frame_ = declare_parameter<std::string>("body_frame", "body_hf");
        fix_roll_deg_ = declare_parameter<double>("fix_roll_deg", -21.30);
        fix_pitch_deg_ = declare_parameter<double>("fix_pitch_deg", 0.0);
        fix_yaw_deg_ = declare_parameter<double>("fix_yaw_deg", 0.0);
        xy_rotation_deg_ = declare_parameter<double>("xy_rotation_deg", 90.0);
        lidar2robot_dis_ = declare_parameter<double>("lidar2robot_dis", 0.31076);
        lidar2robot_ang_ = declare_parameter<double>("lidar2robot_ang", -180.0);
        delta_dis_threshold_ = declare_parameter<double>("delta_dis_threshold", 0.2);
        delta_angle_threshold_ = declare_parameter<double>("delta_angle_threshold", 0.2);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
        base_offset_x_ = declare_parameter<double>("base_offset_x", 0.0);
        base_offset_y_ = declare_parameter<double>("base_offset_y", 0.0);
        base_offset_z_ = declare_parameter<double>("base_offset_z", 0.0);
        base_offset_yaw_deg_ = declare_parameter<double>("base_offset_yaw_deg", 90.0);
        base_offset_pitch_deg_ = declare_parameter<double>("base_offset_pitch_deg", 0.0);

        if (publish_rate_hz_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "publish_rate_hz must be > 0. Forcing to 200.0");
            publish_rate_hz_ = 200.0;
        }

        openSerial();

        q_fix_.setRPY(
            fix_roll_deg_ * M_PI / 180.0,
            fix_pitch_deg_ * M_PI / 180.0,
            fix_yaw_deg_ * M_PI / 180.0);
        q_fix_.normalize();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TfToSerialNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "tf_to_serial_fastlio started.");
        RCLCPP_INFO(get_logger(), "  serial_port: %s", serial_port_.c_str());
        RCLCPP_INFO(get_logger(), "  baudrate: %d", baudrate_);
        RCLCPP_INFO(get_logger(), "  world_frame: %s", world_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  body_frame: %s", body_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  fix_rpy_deg: %.4f, %.4f, %.4f",
                    fix_roll_deg_, fix_pitch_deg_, fix_yaw_deg_);
        RCLCPP_INFO(get_logger(), "  xy_rotation_deg: %.4f", xy_rotation_deg_);
        RCLCPP_INFO(get_logger(), "  lidar2robot_dis: %.6f", lidar2robot_dis_);
        RCLCPP_INFO(get_logger(), "  lidar2robot_ang: %.4f", lidar2robot_ang_);
        RCLCPP_INFO(get_logger(), "  publish_rate_hz: %.2f", publish_rate_hz_);
        RCLCPP_INFO(get_logger(), "  base_offset_xyz: %.3f, %.3f, %.3f",
                    base_offset_x_, base_offset_y_, base_offset_z_);
        RCLCPP_INFO(get_logger(), "  base_offset_yaw_pitch_deg: %.3f, %.3f",
                    base_offset_yaw_deg_, base_offset_pitch_deg_);
    }

    ~TfToSerialNode() {
        if (ser_.isOpen()) {
            ser_.close();
        }
    }

private:
    void openSerial() {
        const auto now = std::chrono::steady_clock::now();
        if (serial_open_ && ser_.isOpen()) {
            return;
        }
        if (now - last_serial_attempt_ < serial_retry_interval_) {
            return;
        }
        last_serial_attempt_ = now;

        try {
            if (ser_.isOpen()) {
                ser_.close();
            }
            ser_.setPort(serial_port_);
            ser_.setBaudrate(static_cast<uint32_t>(baudrate_));
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            ser_.open();
            serial_open_ = true;
        } catch (const serial::IOException& e) {
            serial_open_ = false;
            RCLCPP_ERROR(get_logger(), "Unable to open port: %s", e.what());
        } catch (const std::exception& e) {
            serial_open_ = false;
            RCLCPP_ERROR(get_logger(), "Unexpected error while opening serial port: %s", e.what());
        }

        if (ser_.isOpen()) {
            RCLCPP_INFO(get_logger(), "Serial port initialized.");
            serial_open_ = true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port.");
        }
    }

    void handleSerialFailure(const char* operation, const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Serial %s failed: %s", operation, e.what());
        serial_open_ = false;
        if (ser_.isOpen()) {
            try {
                ser_.close();
            } catch (const std::exception& close_error) {
                RCLCPP_WARN(get_logger(), "Failed to close serial after %s error: %s",
                            operation, close_error.what());
            }
        }
    }

    void drainSerialBytes() {
        if (!ser_.isOpen()) {
            return;
        }

        try {
            const size_t available = ser_.available();
            if (available > 0) {
                const std::string incoming = ser_.read(available);
                rx_buffer_.insert(rx_buffer_.end(), incoming.begin(), incoming.end());
            }
        } catch (const serial::IOException& e) {
            handleSerialFailure("read", e);
        } catch (const std::exception& e) {
            handleSerialFailure("read", e);
        }
    }

    void processSerialOffsetFrames(
        double current_x,
        double current_y,
        double current_z,
        double current_yaw_deg,
        double current_pitch_deg) {
        while (rx_buffer_.size() >= kFrameSize) {
            const std::size_t header_pos = findFrameHeader(rx_buffer_);
            if (header_pos == std::string::npos) {
                if (rx_buffer_.size() > 2) {
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 2);
                }
                return;
            }

            if (header_pos > 0) {
                rx_buffer_.erase(
                    rx_buffer_.begin(),
                    rx_buffer_.begin() + static_cast<std::ptrdiff_t>(header_pos));
            }

            if (rx_buffer_.size() < kFrameSize) {
                return;
            }

            if (!isFrameTailValid(rx_buffer_)) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            std::array<double, kFrameFloatCount> targets = {};
            for (std::size_t i = 0; i < kFrameFloatCount; ++i) {
                float value = 0.0f;
                std::memcpy(
                    &value,
                    rx_buffer_.data() + 3 + i * sizeof(float),
                    sizeof(float));
                targets[i] = static_cast<double>(value);
            }

            serial_anchor_x_ = current_x;
            serial_anchor_y_ = current_y;
            serial_anchor_z_ = current_z;
            serial_anchor_yaw_deg_ = current_yaw_deg;
            serial_anchor_pitch_deg_ = current_pitch_deg;
            serial_target_x_ = targets[0];
            serial_target_y_ = targets[1];
            serial_target_z_ = targets[2];
            serial_target_yaw_deg_ = targets[3];
            serial_target_pitch_deg_ = targets[4];
            serial_anchor_valid_ = true;
            serial_anchor_updated_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Received target frame: x=%.3f y=%.3f z=%.3f yaw=%.3f pitch=%.3f",
                serial_target_x_, serial_target_y_, serial_target_z_,
                serial_target_yaw_deg_, serial_target_pitch_deg_);

            rx_buffer_.erase(
                rx_buffer_.begin(),
                rx_buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameSize));
        }
    }

    static std::size_t findFrameHeader(const std::vector<uint8_t>& buffer) {
        for (std::size_t i = 0; i + 2 < buffer.size(); ++i) {
            if (buffer[i] == kFrameHeader0 &&
                buffer[i + 1] == kFrameHeader1 &&
                buffer[i + 2] == kFrameHeader2) {
                return i;
            }
        }
        return std::string::npos;
    }

    static bool isFrameTailValid(const std::vector<uint8_t>& buffer) {
        return buffer.size() >= kFrameSize &&
               buffer[kFrameSize - 2] == kFrameTail0 &&
               buffer[kFrameSize - 1] == kFrameTail1;
    }

    void timerCallback() {
        try {
            if (!ser_.isOpen()) {
                openSerial();
            }

            drainSerialBytes();

            const geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(world_frame_, body_frame_, tf2::TimePointZero);

            const double raw_x = transform_stamped.transform.translation.x;
            const double raw_y = transform_stamped.transform.translation.y;
            const double raw_z = transform_stamped.transform.translation.z;

            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);

            tf2::Quaternion q_leveled = q_fix_ * q;
            q_leveled.normalize();

            double roll_leveled = 0.0;
            double pitch_leveled = 0.0;
            double yaw_leveled = 0.0;
            tf2::Matrix3x3(q_leveled).getRPY(roll_leveled, pitch_leveled, yaw_leveled);

            if (count_ < WARMUP_FRAMES) {
                sum_x_ += raw_x;
                sum_y_ += raw_y;
                sum_z_ += raw_z;
                sum_sin_roll_leveled_ += std::sin(roll_leveled);
                sum_cos_roll_leveled_ += std::cos(roll_leveled);
                sum_sin_pitch_leveled_ += std::sin(pitch_leveled);
                sum_cos_pitch_leveled_ += std::cos(pitch_leveled);
                sum_sin_yaw_leveled_ += std::sin(yaw_leveled);
                sum_cos_yaw_leveled_ += std::cos(yaw_leveled);
                ++count_;
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 500,
                    "Warming up... (%d/%d)", count_, WARMUP_FRAMES);
                return;
            }

            if (!averaged_) {
                avg_x_ = sum_x_ / WARMUP_FRAMES;
                avg_y_ = sum_y_ / WARMUP_FRAMES;
                avg_z_ = sum_z_ / WARMUP_FRAMES;
                avg_roll_leveled_ = std::atan2(
                    sum_sin_roll_leveled_ / WARMUP_FRAMES,
                    sum_cos_roll_leveled_ / WARMUP_FRAMES);
                avg_pitch_leveled_ = std::atan2(
                    sum_sin_pitch_leveled_ / WARMUP_FRAMES,
                    sum_cos_pitch_leveled_ / WARMUP_FRAMES);
                avg_yaw_leveled_ = std::atan2(
                    sum_sin_yaw_leveled_ / WARMUP_FRAMES,
                    sum_cos_yaw_leveled_ / WARMUP_FRAMES);
                averaged_ = true;

                RCLCPP_INFO(
                    get_logger(),
                    "Warmup done. avg_pos=(%.3f, %.3f, %.3f) avg_leveled_rpy=(roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg)",
                    avg_x_, avg_y_, avg_z_,
                    avg_roll_leveled_ * 180.0 / M_PI,
                    avg_pitch_leveled_ * 180.0 / M_PI,
                    avg_yaw_leveled_ * 180.0 / M_PI);
            }

            const double x = raw_x - avg_x_;
            const double y = raw_y - avg_y_;
            const double z = raw_z - avg_z_;

            const tf2::Vector3 t_li(
                MID360_IMU2LIDAR_X,
                MID360_IMU2LIDAR_Y,
                MID360_IMU2LIDAR_Z);
            const tf2::Vector3 correction = tf2::quatRotate(q, t_li);

            const double x_imu2lidar_local = x - correction.x();
            const double y_imu2lidar_local = y - correction.y();
            const double z_imu2lidar_local = z - correction.z();

            tf2::Vector3 p_vec(x_imu2lidar_local, y_imu2lidar_local, z_imu2lidar_local);
            p_vec = tf2::quatRotate(q_fix_, p_vec);

            x_imu2lidar_ = p_vec.x();
            y_imu2lidar_ = p_vec.y();
            z_imu2lidar_ = p_vec.z();

            const double roll_out = roll_leveled - avg_roll_leveled_;
            const double pitch_out = pitch_leveled - avg_pitch_leveled_;
            const double yaw_out = yaw_leveled - avg_yaw_leveled_;

            const double current_roll_deg = roll_out * 180.0 / M_PI;
            const double current_pitch_deg = pitch_out * 180.0 / M_PI;
            double current_yaw_deg = yaw_out * 180.0 / M_PI;
            current_yaw_deg = wrapTo180(current_yaw_deg);

            processSerialOffsetFrames(
                x_lidar2robot_,
                y_lidar2robot_,
                z_lidar2robot_,
                current_yaw_deg,
                current_pitch_deg);

            const double send_x = computeSendValue(
                x_lidar2robot_, serial_anchor_x_, serial_target_x_, base_offset_x_, serial_anchor_valid_);
            const double send_y = computeSendValue(
                y_lidar2robot_, serial_anchor_y_, serial_target_y_, base_offset_y_, serial_anchor_valid_);
            const double send_z = computeSendValue(
                z_lidar2robot_, serial_anchor_z_, serial_target_z_, base_offset_z_, serial_anchor_valid_);
            const double send_pitch_deg = computeSendValue(
                current_pitch_deg, serial_anchor_pitch_deg_, serial_target_pitch_deg_,
                base_offset_pitch_deg_, serial_anchor_valid_);
            const double send_yaw_deg = computeSendYawDeg(
                current_yaw_deg, serial_anchor_yaw_deg_, serial_target_yaw_deg_,
                base_offset_yaw_deg_, serial_anchor_valid_);
            const bool anchor_updated = serial_anchor_updated_;
            serial_anchor_updated_ = false;

            const double xy_rot_rad = xy_rotation_deg_ * M_PI / 180.0;
            const double lidar_ang_rad = lidar2robot_ang_ * M_PI / 180.0;

            double x_pos = std::cos(xy_rot_rad) * x_imu2lidar_ - std::sin(xy_rot_rad) * y_imu2lidar_;
            double y_pos = std::sin(xy_rot_rad) * x_imu2lidar_ + std::cos(xy_rot_rad) * y_imu2lidar_;
            double z_pos = z_imu2lidar_;

            if (is_first_) {
                first_x_ = x_pos;
                first_y_ = y_pos;
                first_z_ = z_pos;
            }
            x_pos -= first_x_;
            y_pos -= first_y_;
            z_pos -= first_z_;

            const double effective_yaw = yaw_out + xy_rot_rad;
            x_lidar2robot_ = x_pos - lidar2robot_dis_ * std::cos(lidar_ang_rad + effective_yaw);
            y_lidar2robot_ = y_pos - lidar2robot_dis_ * std::sin(lidar_ang_rad + effective_yaw);
            z_lidar2robot_ = z_pos + LIDAR2ROBOT_Z;

            if (is_first_) {
                output_first_x_ = x_lidar2robot_;
                output_first_y_ = y_lidar2robot_;
                output_first_z_ = z_lidar2robot_;
                RCLCPP_INFO(get_logger(), "Output origin captured: x=%.4f y=%.4f z=%.4f",
                            output_first_x_, output_first_y_, output_first_z_);
            }
            x_lidar2robot_ -= output_first_x_;
            y_lidar2robot_ -= output_first_y_;
            z_lidar2robot_ -= output_first_z_;

            if (!is_first_ && !anchor_updated) {
                const double dx = x_lidar2robot_ - last_x_;
                const double dy = y_lidar2robot_ - last_y_;
                const double dz = z_lidar2robot_ - last_z_;
                const double ddist = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (ddist > delta_dis_threshold_) {
                    RCLCPP_WARN(get_logger(),
                                "Position jump: dx=%.4f dy=%.4f dz=%.4f dist=%.4f (thr=%.4f)",
                                dx, dy, dz, ddist, delta_dis_threshold_);
                }

                const double dangle_deg = shortestAngleDeltaDeg(send_yaw_deg, last_angle_deg_);
                if (std::fabs(dangle_deg) > delta_angle_threshold_) {
                    RCLCPP_WARN(get_logger(), "Angle jump: dangle=%.4f deg (thr=%.4f)",
                                dangle_deg, delta_angle_threshold_);
                }
            }

            last_x_ = send_x;
            last_y_ = send_y;
            last_z_ = send_z;
            last_angle_deg_ = send_yaw_deg;
            is_first_ = false;

            if (!anchor_updated) {
                const double delta_deg = send_yaw_deg - euler_last_deg_;
                if (delta_deg > 180.0) {
                    --k_;
                } else if (delta_deg < -180.0) {
                    ++k_;
                }
            }
            euler_total_ = send_yaw_deg + 360.0 * k_;
            euler_last_deg_ = send_yaw_deg;

            if (ser_.isOpen()) {
                // 只有串口真正在线时，才持续输出和下发当前姿态/位置
                RCLCPP_INFO(get_logger(), "x=%.3f y=%.3f z=%.3f yaw=%.3f pitch=%.3f roll=%.3f",
                            send_y, -send_x, send_z,
                            send_yaw_deg, send_pitch_deg, current_roll_deg);

                std::vector<uint8_t> frame;
                frame.reserve(kFrameSize);
                frame.push_back(kFrameHeader0);
                frame.push_back(kFrameHeader1);
                frame.push_back(kFrameHeader2);
                appendFloat(frame, static_cast<float>(send_y));
                appendFloat(frame, static_cast<float>(-send_x));
                appendFloat(frame, static_cast<float>(send_z));
                appendFloat(frame, static_cast<float>(send_yaw_deg));
                appendFloat(frame, static_cast<float>(send_pitch_deg));
                frame.push_back(kFrameTail0);
                frame.push_back(kFrameTail1);

                try {
                    ser_.write(frame);
                } catch (const serial::IOException& e) {
                    handleSerialFailure("write", e);
                } catch (const std::exception& e) {
                    handleSerialFailure("write", e);
                }
            } else {
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Serial disconnected, retrying open on %s", serial_port_.c_str());
            }

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", ex.what());
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(get_logger(), "Unexpected error in timer callback: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial ser_;

    std::string serial_port_;
    int baudrate_ = 115200;
    std::string world_frame_;
    std::string body_frame_;

    double fix_roll_deg_ = 21.30;
    double fix_pitch_deg_ = 0.0;
    double fix_yaw_deg_ = 0.0;
    double xy_rotation_deg_ = 90.0;
    double lidar2robot_dis_ = 0.31076;
    double lidar2robot_ang_ = 0.0;
    double delta_dis_threshold_ = 0.2;
    double delta_angle_threshold_ = 0.2;
    double publish_rate_hz_ = 200.0;
    double base_offset_x_ = 0.0;
    double base_offset_y_ = 0.0;
    double base_offset_z_ = 0.0;
    double base_offset_yaw_deg_ = -90.0;
    double base_offset_pitch_deg_ = 0.0;
    bool serial_open_ = false;
    std::chrono::steady_clock::time_point last_serial_attempt_{};
    std::chrono::steady_clock::duration serial_retry_interval_{std::chrono::seconds(1)};

    bool serial_anchor_valid_ = false;
    bool serial_anchor_updated_ = false;
    double serial_anchor_x_ = 0.0;
    double serial_anchor_y_ = 0.0;
    double serial_anchor_z_ = 0.0;
    double serial_anchor_yaw_deg_ = 0.0;
    double serial_anchor_pitch_deg_ = 0.0;
    double serial_target_x_ = 0.0;
    double serial_target_y_ = 0.0;
    double serial_target_z_ = 0.0;
    double serial_target_yaw_deg_ = 0.0;
    double serial_target_pitch_deg_ = 0.0;

    tf2::Quaternion q_fix_;

    int count_ = 0;
    bool averaged_ = false;
    double sum_x_ = 0.0;
    double sum_y_ = 0.0;
    double sum_z_ = 0.0;
    double sum_sin_roll_leveled_ = 0.0;
    double sum_cos_roll_leveled_ = 0.0;
    double sum_sin_pitch_leveled_ = 0.0;
    double sum_cos_pitch_leveled_ = 0.0;
    double sum_sin_yaw_leveled_ = 0.0;
    double sum_cos_yaw_leveled_ = 0.0;

    double avg_x_ = 0.0;
    double avg_y_ = 0.0;
    double avg_z_ = 0.0;
    double avg_roll_leveled_ = 0.0;
    double avg_pitch_leveled_ = 0.0;
    double avg_yaw_leveled_ = 0.0;

    bool is_first_ = true;
    double first_x_ = 0.0;
    double first_y_ = 0.0;
    double first_z_ = 0.0;
    double output_first_x_ = 0.0;
    double output_first_y_ = 0.0;
    double output_first_z_ = 0.0;

    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_z_ = 0.0;
    double last_angle_deg_ = 0.0;
    double euler_last_deg_ = 0.0;
    double euler_total_ = 0.0;
    int k_ = 0;

    double x_imu2lidar_ = 0.0;
    double y_imu2lidar_ = 0.0;
    double z_imu2lidar_ = 0.0;
    double x_lidar2robot_ = 0.0;
    double y_lidar2robot_ = 0.0;
    double z_lidar2robot_ = 0.0;

    std::vector<uint8_t> rx_buffer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfToSerialNode>());
    rclcpp::shutdown();
    return 0;
}
