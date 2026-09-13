#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <serial/serial.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/wait.h>

namespace {

constexpr std::size_t kFrameFloatCount = 5;
constexpr std::array<uint8_t, 3> kDefaultFrameHeader = {0xFF, 0xFE, 0x01};
constexpr std::array<uint8_t, 2> kDefaultFrameTail = {0xAA, 0xDD};
constexpr std::array<uint8_t, 7> kDefaultRestartMagic = {
    0xFF, 0xFE, 0x01, 0x78, 0x13, 0xAA, 0xDD};
constexpr std::size_t kFrameSize =
    kDefaultFrameHeader.size() + kFrameFloatCount * sizeof(float) + kDefaultFrameTail.size();
static_assert(kFrameSize == 25, "The receiver protocol requires a 25-byte packet.");

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

std::string getEnvString(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::string(value) : std::string();
}

template <std::size_t N>
bool assignByteArray(
    const std::vector<int64_t>& values,
    std::array<uint8_t, N>& destination) {
    if (values.size() != N) {
        return false;
    }

    for (std::size_t i = 0; i < N; ++i) {
        if (values[i] < 0 || values[i] > 0xFF) {
            return false;
        }
        destination[i] = static_cast<uint8_t>(values[i]);
    }
    return true;
}

bool containsSequence(
    const std::vector<uint8_t>& buffer,
    const std::array<uint8_t, 7>& sequence) {
    if (buffer.size() < sequence.size()) {
        return false;
    }

    return std::search(
               buffer.begin(), buffer.end(),
               sequence.begin(), sequence.end()) != buffer.end();
}

}  // namespace

class TfToSerialNode : public rclcpp::Node {
public:
    TfToSerialNode()
        : Node("tf_to_serial_fastlio") {
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/my_lidar");
        baudrate_ = declare_parameter<int>("baudrate", 115200);
        serial_timeout_ms_ = declare_parameter<int>("serial_timeout_ms", 1000);
        serial_retry_interval_ms_ = declare_parameter<int>("serial_retry_interval_ms", 1000);
        world_frame_ = declare_parameter<std::string>("world_frame", "odom");
        body_frame_ = declare_parameter<std::string>("body_frame", "base_link_hf");
        delta_dis_threshold_ = declare_parameter<double>("delta_dis_threshold", 0.2);
        delta_angle_threshold_ = declare_parameter<double>("delta_angle_threshold", 0.2);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
        high_freq_info_enabled_ = declare_parameter<bool>("high_freq_info_enabled", false);
        restart_command_ = declare_parameter<std::string>(
            "restart_command",
            getEnvString("POSTION_ODOM_RESTART_CMD"));
        tx_position_scale_ = declare_parameter<double>("tx_position_scale", 1000.0);

        const auto tx_header = declare_parameter<std::vector<int64_t>>(
            "tx_header", std::vector<int64_t>{0xFF, 0xFE, 0x01});
        const auto tx_tail = declare_parameter<std::vector<int64_t>>(
            "tx_tail", std::vector<int64_t>{0xAA, 0xDD});
        const auto restart_magic = declare_parameter<std::vector<int64_t>>(
            "restart_magic", std::vector<int64_t>{0xFF, 0xFE, 0x01, 0x78, 0x13, 0xAA, 0xDD});

        if (!assignByteArray(tx_header, tx_header_)) {
            RCLCPP_WARN(get_logger(), "tx_header must contain exactly three bytes; using FF FE 01.");
            tx_header_ = kDefaultFrameHeader;
        }
        if (!assignByteArray(tx_tail, tx_tail_)) {
            RCLCPP_WARN(get_logger(), "tx_tail must contain exactly two bytes; using AA DD.");
            tx_tail_ = kDefaultFrameTail;
        }
        if (!assignByteArray(restart_magic, restart_magic_)) {
            RCLCPP_WARN(
                get_logger(),
                "restart_magic must contain exactly seven bytes; using FF FE 01 78 13 AA DD.");
            restart_magic_ = kDefaultRestartMagic;
        }
        if (publish_rate_hz_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "publish_rate_hz must be > 0. Forcing to 200.0");
            publish_rate_hz_ = 200.0;
        }
        if (baudrate_ <= 0) {
            RCLCPP_WARN(get_logger(), "baudrate must be > 0. Forcing to 115200.");
            baudrate_ = 115200;
        }
        if (serial_timeout_ms_ <= 0) {
            RCLCPP_WARN(get_logger(), "serial_timeout_ms must be > 0. Forcing to 1000.");
            serial_timeout_ms_ = 1000;
        }
        if (serial_retry_interval_ms_ <= 0) {
            RCLCPP_WARN(get_logger(), "serial_retry_interval_ms must be > 0. Forcing to 1000.");
            serial_retry_interval_ms_ = 1000;
        }
        if (!std::isfinite(tx_position_scale_) || tx_position_scale_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "tx_position_scale must be finite and > 0. Forcing to 1000.");
            tx_position_scale_ = 1000.0;
        }
        if (!std::isfinite(delta_dis_threshold_) || delta_dis_threshold_ < 0.0) {
            RCLCPP_WARN(get_logger(), "delta_dis_threshold must be finite and >= 0. Forcing to 0.2.");
            delta_dis_threshold_ = 0.2;
        }
        if (!std::isfinite(delta_angle_threshold_) || delta_angle_threshold_ < 0.0) {
            RCLCPP_WARN(get_logger(), "delta_angle_threshold must be finite and >= 0. Forcing to 0.2.");
            delta_angle_threshold_ = 0.2;
        }
        if (serial_port_.empty()) {
            throw std::invalid_argument("serial_port must not be empty");
        }
        if (world_frame_.empty() || body_frame_.empty() || world_frame_ == body_frame_ ||
            world_frame_.front() == '/' || body_frame_.front() == '/') {
            throw std::invalid_argument(
                "world_frame and body_frame must be distinct, non-empty TF names without leading '/'");
        }
        openSerial();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TfToSerialNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "tf_to_serial_fastlio started.");
        RCLCPP_INFO(get_logger(), "  serial_port: %s", serial_port_.c_str());
        RCLCPP_INFO(get_logger(), "  baudrate: %d", baudrate_);
        RCLCPP_INFO(get_logger(), "  serial_timeout_ms: %d", serial_timeout_ms_);
        RCLCPP_INFO(get_logger(), "  serial_retry_interval_ms: %d", serial_retry_interval_ms_);
        RCLCPP_INFO(get_logger(), "  world_frame: %s", world_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  body_frame: %s", body_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  publish_rate_hz: %.2f", publish_rate_hz_);
        RCLCPP_INFO(get_logger(), "  high_freq_info_enabled: %s",
                    high_freq_info_enabled_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  restart_command: %s",
                    restart_command_.empty() ? "<empty>" : restart_command_.c_str());
        RCLCPP_INFO(get_logger(), "  tx_position_scale: %.3f", tx_position_scale_);
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
        if (now - last_serial_attempt_ < std::chrono::milliseconds(serial_retry_interval_ms_)) {
            return;
        }
        last_serial_attempt_ = now;

        try {
            if (ser_.isOpen()) {
                ser_.close();
            }
            ser_.setPort(serial_port_);
            ser_.setBaudrate(static_cast<uint32_t>(baudrate_));
            serial::Timeout timeout = serial::Timeout::simpleTimeout(
                static_cast<uint32_t>(serial_timeout_ms_));
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

    void requestRestartIfNeeded() {
        if (restart_requested_) {
            return;
        }

        if (!containsSequence(rx_buffer_, restart_magic_)) {
            // Preserve a magic-frame prefix split across reads, but never retain an
            // unbounded amount of unrelated incoming data.
            constexpr std::size_t kBytesToKeep = kDefaultRestartMagic.size() - 1;
            if (rx_buffer_.size() > kBytesToKeep) {
                rx_buffer_.erase(
                    rx_buffer_.begin(),
                    rx_buffer_.end() - static_cast<std::ptrdiff_t>(kBytesToKeep));
            }
            return;
        }

        rx_buffer_.clear();

        RCLCPP_WARN(
            get_logger(),
            "Received configured restart magic frame, requesting postion_odom restart.");

        if (restart_command_.empty()) {
            RCLCPP_ERROR(
                get_logger(),
                "restart_command parameter is empty, cannot restart postion_odom.sh automatically.");
            return;
        }

        const std::string detached_command = "gnome-terminal -- bash -lc \"" + restart_command_ + " </dev/null >/tmp/postion_odom_restart.log 2>&1\"";
        const int rc = std::system(detached_command.c_str());
        if (rc == -1) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to invoke restart command: %s",
                detached_command.c_str());
            return;
        }

        if (WIFEXITED(rc) && WEXITSTATUS(rc) == 0) {
            restart_requested_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Restart command launched successfully. Log: /tmp/postion_odom_restart.log");
        } else {
            RCLCPP_ERROR(
                get_logger(),
                "Restart command returned non-zero status: raw=%d cmd=%s",
                rc, detached_command.c_str());
        }
    }

    void timerCallback() {
        try {
            if (!ser_.isOpen()) {
                openSerial();
            }

            drainSerialBytes();
            requestRestartIfNeeded();
            if (restart_requested_) {
                rclcpp::shutdown();
                return;
            }

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

            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // FAST-LIO already publishes robot-centered, zero-origin odometry.
            // Serialize that TF directly so this node never creates a second origin.
            const double send_x = raw_x;
            const double send_y = raw_y;
            const double send_z = raw_z;
            const double current_roll_deg = wrapTo180(roll * 180.0 / M_PI);
            const double send_pitch_deg = wrapTo180(pitch * 180.0 / M_PI);
            const double send_yaw_deg = wrapTo180(yaw * 180.0 / M_PI);

            if (!is_first_) {
                const double dx = send_x - last_x_;
                const double dy = send_y - last_y_;
                const double dz = send_z - last_z_;
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

            if (ser_.isOpen()) {
                if (high_freq_info_enabled_) {
                    RCLCPP_INFO(get_logger(),
                                "x=%.3f y=%.3f z=%9.3f yaw=%.3f pitch=%.3f roll=%.3f",
                                send_x, send_y, send_z,
                                send_yaw_deg, send_pitch_deg, current_roll_deg);
                } else {
                    // 默认降频，避免长期运行时被日志刷盘拖慢。
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "x=%.3f y=%.3f z=%.3f yaw=%.3f pitch=%.3f roll=%.3f",
                        send_x, send_y, send_z,
                        send_yaw_deg, send_pitch_deg, current_roll_deg);
                }

                std::vector<uint8_t> frame;
                frame.reserve(kFrameSize);
                frame.insert(frame.end(), tx_header_.begin(), tx_header_.end());
                appendFloat(frame, static_cast<float>(send_x * tx_position_scale_));
                appendFloat(frame, static_cast<float>(send_y * tx_position_scale_));
                appendFloat(frame, static_cast<float>(send_z * tx_position_scale_));
                appendFloat(frame, static_cast<float>(send_yaw_deg));
                appendFloat(frame, static_cast<float>(send_pitch_deg));
                frame.insert(frame.end(), tx_tail_.begin(), tx_tail_.end());

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
    int serial_timeout_ms_ = 1000;
    int serial_retry_interval_ms_ = 1000;
    std::string world_frame_;
    std::string body_frame_;

    double delta_dis_threshold_ = 0.2;
    double delta_angle_threshold_ = 0.2;
    double publish_rate_hz_ = 200.0;
    bool high_freq_info_enabled_ = false;
    double tx_position_scale_ = 1000.0;
    std::array<uint8_t, 3> tx_header_ = kDefaultFrameHeader;
    std::array<uint8_t, 2> tx_tail_ = kDefaultFrameTail;
    std::array<uint8_t, 7> restart_magic_ = kDefaultRestartMagic;
    bool serial_open_ = false;
    std::chrono::steady_clock::time_point last_serial_attempt_{};
    std::string restart_command_;
    bool restart_requested_ = false;

    bool is_first_ = true;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_z_ = 0.0;
    double last_angle_deg_ = 0.0;

    std::vector<uint8_t> rx_buffer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfToSerialNode>());
    rclcpp::shutdown();
    return 0;
}
