// One decoded frame produces one sample. No cross-frame measurement cache.
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <hipnuc_msgs/msg/hipnuc_imu.hpp>

#include "hipnuc_serial.h"
#include "hipnuc_convert.hpp"

using SteadyClock = std::chrono::steady_clock;

namespace {

// Gregorian calendar date to days since 1970-01-01. This avoids process-wide
// timezone state and keeps the device UTC conversion independent of local time.
int64_t days_from_civil(int year, unsigned month, unsigned day)
{
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
    const unsigned adjusted_month = month > 2 ? month - 3 : month + 9;
    const unsigned day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    const unsigned day_of_era =
        year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    return static_cast<int64_t>(era) * 146097 + day_of_era - 719468;
}

bool device_utc_stamp(const hipnuc_sample_t &sample, rclcpp::Time &stamp)
{
    if (!(sample.valid & HIPNUC_VALID_UTC) || !hipnuc_utc_is_valid(&sample.utc)) return false;

    const auto &utc = sample.utc;
    const int64_t seconds =
        days_from_civil(utc.year, utc.month, utc.day) * 86400 +
        static_cast<int64_t>(utc.hour) * 3600 +
        static_cast<int64_t>(utc.minute) * 60 + utc.second;
    const int64_t nanoseconds = seconds * 1000000000LL +
        static_cast<int64_t>(utc.millisecond) * 1000000LL;
    stamp = rclcpp::Time(nanoseconds, RCL_SYSTEM_TIME);
    return true;
}

}  // namespace

class SerialNode : public rclcpp::Node {
public:
    SerialNode() : Node("hipnuc_serial")
    {
        rcl_interfaces::msg::ParameterDescriptor startup;
        startup.read_only = true;
        startup.description = "Applied at startup; restart the node to change this parameter.";
        port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0", startup);
        baudrate_ = declare_parameter<int>("baudrate", 115200, startup);
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link", startup);
        publish_imu_ = declare_parameter<bool>("publish_imu", true, startup);
        publish_mag_ = declare_parameter<bool>("publish_mag", true, startup);
        publish_temperature_ = declare_parameter<bool>("publish_temperature", true, startup);
        publish_hipnuc_ = declare_parameter<bool>("publish_hipnuc", true, startup);
        timestamp_source_ = declare_parameter<std::string>("timestamp_source", "host", startup);
        if (port_.empty() || baudrate_ <= 0 || frame_id_.empty())
            throw std::invalid_argument("port/frame_id must be nonempty and baudrate positive");
        if (timestamp_source_ != "host" && timestamp_source_ != "device_utc")
            throw std::invalid_argument("timestamp_source must be 'host' or 'device_utc'");
        rclcpp::PublisherOptions publisher_options;
        // Let a deployment relax reliability or depth without rebuilding, e.g.
        // qos_overrides./imu/data.publisher.reliability:=best_effort.
        publisher_options.qos_overriding_options =
            rclcpp::QosOverridingOptions::with_default_policies();
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 100, publisher_options);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100, publisher_options);
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10, publisher_options);
        full_pub_ = create_publisher<hipnuc_msgs::msg::HipnucImu>("hipnuc/imu", 100, publisher_options);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        RCLCPP_INFO(
            get_logger(),
            "Requires device ENU output configuration; timestamp_source=%s.",
            timestamp_source_.c_str());
    }

    ~SerialNode() { hipnuc_serial_close(&serial_); }

    void run()
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(shared_from_this());
        auto last_diag = SteadyClock::now();
        auto next_retry = last_diag;
        auto next_warning = last_diag;
        while (rclcpp::ok()) {
            auto current = SteadyClock::now();
            if (!serial_.is_open && current >= next_retry) {
                const bool opened = hipnuc_serial_open(&serial_, port_.c_str(), baudrate_) == 0;
                next_retry = current + std::chrono::seconds(1);
                if (opened) {
                    received_sample_ = false;
                    bytes_at_last_diag_ = 0;
                    RCLCPP_INFO(get_logger(), "opened %s at %d baud", port_.c_str(), baudrate_);
                } else if (current >= next_warning) {
                    RCLCPP_WARN(get_logger(), "cannot open %s: %s", port_.c_str(), hipnuc_serial_last_error(&serial_));
                    next_warning = current + std::chrono::seconds(5);
                }
            }
            if (serial_.is_open) {
                hipnuc_sample_t sample;
                const int result = hipnuc_serial_read_sample(&serial_, &sample, 50);
                if (result > 0) publish(sample);
                if (result < 0) {
                    RCLCPP_ERROR(get_logger(), "%s read failed: %s; reopening", port_.c_str(), hipnuc_serial_last_error(&serial_));
                    hipnuc_serial_close(&serial_);
                    received_sample_ = false;
                    next_retry = SteadyClock::now() + std::chrono::seconds(1);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            // Failure and idle paths must still service ROS and diagnostics.
            executor.spin_some();
            current = SteadyClock::now();
            const double elapsed = std::chrono::duration<double>(current - last_diag).count();
            if (elapsed >= 1.0) {
                publish_diagnostics(current, elapsed);
                last_diag = current;
            }
        }
    }

private:
    void publish(const hipnuc_sample_t &s)
    {
        const auto host_stamp = now();
        auto stamp = host_stamp;
        const bool sample_has_imu = hipnuc_ros::has_imu(s);
        const bool sample_device_utc_valid =
            (s.valid & HIPNUC_VALID_UTC) && hipnuc_utc_is_valid(&s.utc);
        std::string sample_timestamp_source = "host";
        if (timestamp_source_ == "device_utc") {
            if (device_utc_stamp(s, stamp)) {
                sample_timestamp_source = "device_utc";
            } else {
                sample_timestamp_source = "host_fallback";
                if (sample_has_imu) {
                    ++device_utc_fallback_frames_;
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "device_utc requested but this IMU sample has no synchronized UTC; using host ROS time");
                }
            }
        }
        if (sample_has_imu) {
            last_device_utc_valid_ = sample_device_utc_valid;
            last_timestamp_source_ = sample_timestamp_source;
            last_host_minus_stamp_sec_ =
                static_cast<double>(host_stamp.nanoseconds() - stamp.nanoseconds()) * 1e-9;
        }
        ++frames_;
        last_frame_ = SteadyClock::now();
        received_sample_ = true;
        if (publish_imu_ && sample_has_imu) {
            sensor_msgs::msg::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_imu(s, m);
            imu_pub_->publish(m);
        }
        if (publish_mag_ && (s.valid & HIPNUC_VALID_MAG) && hipnuc_ros::finite_vector(s.mag)) {
            sensor_msgs::msg::MagneticField m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_mag(s, m);
            mag_pub_->publish(m);
        }
        if (publish_temperature_ && (s.valid & HIPNUC_VALID_TEMPERATURE) && std::isfinite(s.temperature)) {
            sensor_msgs::msg::Temperature m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_temperature(s, m);
            temp_pub_->publish(m);
        }
        if (publish_hipnuc_) {
            hipnuc_msgs::msg::HipnucImu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_hipnuc(s, m);
            full_pub_->publish(m);
        }
    }

    void publish_diagnostics(SteadyClock::time_point current, double elapsed)
    {
        diagnostic_msgs::msg::DiagnosticArray arr;
        diagnostic_msgs::msg::DiagnosticStatus st;
        arr.header.stamp = now();
        st.name = std::string(get_fully_qualified_name()) + ": serial";
        st.hardware_id = port_;
        const double age = received_sample_ ? std::chrono::duration<double>(current - last_frame_).count() : -1.0;
        if (!serial_.is_open) {
            st.level = st.ERROR;
            st.message = "port not open";
        } else if (age < 0.0 || age > 2.0) {
            st.level = st.WARN;
            st.message = serial_.bytes_received > bytes_at_last_diag_ ? "bytes received but no valid frames (check baudrate/output)" : "no data";
        } else {
            st.level = st.OK;
            st.message = "receiving";
        }
        // Nothing else reaches the console after a successful open: report each change.
        if (st.level != last_level_) {
            last_level_ = st.level;
            if (st.level == st.OK) RCLCPP_INFO(get_logger(), "%s", st.message.c_str());
            else RCLCPP_WARN(get_logger(), "%s", st.message.c_str());
        }
        auto kv = [&](const char *key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key;
            entry.value = value;
            st.values.push_back(entry);
        };
        kv("frames", std::to_string(frames_));
        kv("frame_rate_hz", std::to_string((frames_ - frames_at_last_diag_) / elapsed));
        kv("connection_bytes", std::to_string(serial_.bytes_received));
        kv("connection_crc_errors", std::to_string(serial_.binary.crc_error_count));
        kv("connection_receive_errors", std::to_string(serial_.receive_errors));
        kv("connection_invalid_frames", std::to_string(serial_.binary.invalid_count));
        kv("connection_nmea_checksum_errors", std::to_string(serial_.nmea.checksum_error_count));
        kv("timestamp_source_config", timestamp_source_);
        kv("last_timestamp_source", last_timestamp_source_);
        kv("device_utc_valid", last_device_utc_valid_ ? "true" : "false");
        kv("device_utc_fallback_frames", std::to_string(device_utc_fallback_frames_));
        kv("host_minus_stamp_sec", std::to_string(last_host_minus_stamp_sec_));
        kv("last_error", hipnuc_serial_last_error(&serial_));
        bytes_at_last_diag_ = serial_.bytes_received;
        frames_at_last_diag_ = frames_;
        arr.status.push_back(st);
        diag_pub_->publish(arr);
    }

    std::string port_, frame_id_, timestamp_source_;
    int baudrate_;
    bool publish_imu_, publish_mag_, publish_temperature_, publish_hipnuc_;
    hipnuc_serial_t serial_{};
    uint64_t frames_ = 0, frames_at_last_diag_ = 0;
    uint64_t bytes_at_last_diag_ = 0;
    uint64_t device_utc_fallback_frames_ = 0;
    bool received_sample_ = false;
    bool last_device_utc_valid_ = false;
    int last_level_ = -1;
    double last_host_minus_stamp_sec_ = 0.0;
    std::string last_timestamp_source_ = "none";
    SteadyClock::time_point last_frame_{};
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<hipnuc_msgs::msg::HipnucImu>::SharedPtr full_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SerialNode>();
        node->run();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("hipnuc"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
