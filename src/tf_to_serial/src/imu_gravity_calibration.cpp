#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

class IMUGravityCalibration : public rclcpp::Node {
public:
    IMUGravityCalibration() : Node("imu_gravity_calibration") {
        // 订阅 IMU 话题
        auto imu_callback = [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            this->imu_callback(msg);
        };
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 50, imu_callback);

        RCLCPP_INFO(this->get_logger(), 
            "IMU Gravity Calibration Tool Started");
        RCLCPP_INFO(this->get_logger(), 
            "====================================");
        RCLCPP_INFO(this->get_logger(), 
            "Instructions:");
        RCLCPP_INFO(this->get_logger(), 
            "1. Place the device on a stable, level surface");
        RCLCPP_INFO(this->get_logger(), 
            "2. Keep it still for at least 5 seconds");
        RCLCPP_INFO(this->get_logger(), 
            "3. The calibration will automatically complete");
        RCLCPP_INFO(this->get_logger(), 
            "4. Results will be printed below");
        RCLCPP_INFO(this->get_logger(), 
            "====================================\n");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 如果已经标定完成，忽略后续数据
        if (calibration_complete_) {
            return;
        }

        // 累积加速度数据
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        accel_buffer_.push_back({ax, ay, az});
        sample_count_++;

        // 如果 IMU 提供了 orientation 信息，则累积 yaw（航向）用于稳定估计
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        // yaw (z-axis rotation) from quaternion
        double yaw_rad = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        sum_sin_yaw_ += std::sin(yaw_rad);
        sum_cos_yaw_ += std::cos(yaw_rad);
        
        // 打印进度
        if (sample_count_ % 100 == 0) {
            std::cout << "\r[Progress] Collected " << sample_count_ 
                      << " samples (need ~2000 for stable calibration)     " << std::flush;
        }

        // 需要足够的数据样本（2000 个样本 @ 200Hz ≈ 10 秒）
        if (accel_buffer_.size() < 10000) {
            return;
        }

        // 计算平均加速度
        double avg_ax = 0.0, avg_ay = 0.0, avg_az = 0.0;
        for (const auto& accel : accel_buffer_) {
            avg_ax += accel[0];
            avg_ay += accel[1];
            avg_az += accel[2];
        }
        avg_ax /= accel_buffer_.size();
        avg_ay /= accel_buffer_.size();
        avg_az /= accel_buffer_.size();

        // 计算加速度方差（判断数据稳定性）
        double variance_x = 0.0, variance_y = 0.0, variance_z = 0.0;
        for (const auto& accel : accel_buffer_) {
            variance_x += (accel[0] - avg_ax) * (accel[0] - avg_ax);
            variance_y += (accel[1] - avg_ay) * (accel[1] - avg_ay);
            variance_z += (accel[2] - avg_az) * (accel[2] - avg_az);
        }
        variance_x = std::sqrt(variance_x / accel_buffer_.size());
        variance_y = std::sqrt(variance_y / accel_buffer_.size());
        variance_z = std::sqrt(variance_z / accel_buffer_.size());
        
        double total_stddev = std::sqrt(variance_x*variance_x + variance_y*variance_y + variance_z*variance_z);

        // 检查数据是否足够稳定（提高精度要求）
        if (total_stddev > 0.08) {
            RCLCPP_WARN(this->get_logger(),
                "IMU data unstable (stddev=%.6f). Please keep device still. (threshold: 0.08)", 
                total_stddev);
            // 清空缓冲区重新采集
            accel_buffer_.clear();
            sample_count_ = 0;
            sum_sin_yaw_ = 0.0;
            sum_cos_yaw_ = 0.0;
            return;
        }

        // 如果数据稳定，计算重力对齐角度
        double accel_norm = std::sqrt(avg_ax*avg_ax + avg_ay*avg_ay + avg_az*avg_az);
        
        // 根据加速度计算欧拉角
        // pitch 角 = atan2(ax, sqrt(ay^2 + az^2))
        // roll 角 = atan2(ay, az)
        double pitch_rad = std::atan2(avg_ax, std::sqrt(avg_ay*avg_ay + avg_az*avg_az));
        double roll_rad = std::atan2(avg_ay, avg_az);

        double pitch_deg = pitch_rad * 180.0 / M_PI;
        double roll_deg = roll_rad * 180.0 / M_PI;

        // 平均 yaw（使用 sin/cos 平均以避免角度跳变）
        double avg_yaw_rad = 0.0;
        double avg_yaw_deg = 0.0;
        if (accel_buffer_.size() > 0) {
            double sin_mean = sum_sin_yaw_ / accel_buffer_.size();
            double cos_mean = sum_cos_yaw_ / accel_buffer_.size();
            avg_yaw_rad = std::atan2(sin_mean, cos_mean);
            avg_yaw_deg = avg_yaw_rad * 180.0 / M_PI;
        }

        // 打印详细的标定结果
        std::cout << "\n\n";
        RCLCPP_INFO(this->get_logger(),
            "========== IMU GRAVITY ALIGNMENT CALIBRATION RESULTS ==========");
        
        RCLCPP_INFO(this->get_logger(),
            "Raw Accelerations (avg):");
        RCLCPP_INFO(this->get_logger(),
            "  ax = %.6f m/s²", avg_ax);
        RCLCPP_INFO(this->get_logger(),
            "  ay = %.6f m/s²", avg_ay);
        RCLCPP_INFO(this->get_logger(),
            "  az = %.6f m/s²", avg_az);
        RCLCPP_INFO(this->get_logger(),
            "  norm = %.6f m/s² (should be ~9.807)", accel_norm);
        
        RCLCPP_INFO(this->get_logger(),
            "Stability Analysis:");
        RCLCPP_INFO(this->get_logger(),
            "  σ(ax) = %.6f m/s²", variance_x);
        RCLCPP_INFO(this->get_logger(),
            "  σ(ay) = %.6f m/s²", variance_y);
        RCLCPP_INFO(this->get_logger(),
            "  σ(az) = %.6f m/s²", variance_z);
        RCLCPP_INFO(this->get_logger(),
            "  total_stddev = %.6f (threshold: 0.08)", total_stddev);
        RCLCPP_INFO(this->get_logger(),
            "  Samples collected: %zu", accel_buffer_.size());
        
        RCLCPP_INFO(this->get_logger(),
            "Derived Gravity Alignment Angles (to align gravity to plane):");
        RCLCPP_INFO(this->get_logger(),
            "  roll  = %.4f° (%.6f rad)", roll_deg, roll_rad);
        RCLCPP_INFO(this->get_logger(),
            "  pitch = %.4f° (%.6f rad)", pitch_deg, pitch_rad);
        RCLCPP_INFO(this->get_logger(),
            "  yaw   = %.4f° (%.6f rad)  // averaged from IMU orientation samples", avg_yaw_deg, avg_yaw_rad);
        
        RCLCPP_INFO(this->get_logger(),
            "========== RECOMMENDED CODE UPDATE ==========");
        RCLCPP_INFO(this->get_logger(),
            "Replace the following line in tf_to_serial_node.cpp with full RPY compensation:");
        RCLCPP_INFO(this->get_logger(),
            "  OLD: q_fix_.setRPY(0.0, -(180 - 56.39) * M_PI / 180.0, 0.0);");
        RCLCPP_INFO(this->get_logger(),
            "  NEW: q_fix_.setRPY(%.6f, %.6f, %.6f);  // roll, pitch, yaw (rad)",
            roll_rad, pitch_rad, avg_yaw_rad);

        RCLCPP_INFO(this->get_logger(),
            "Or in setRPY format with degrees:");
        RCLCPP_INFO(this->get_logger(),
            "  q_fix_.setRPY(%.4f°, %.4f°, %.4f°)  // roll, pitch, yaw", roll_deg, pitch_deg, avg_yaw_deg);
        
        RCLCPP_INFO(this->get_logger(),
            "==============================================");
        RCLCPP_INFO(this->get_logger(),
            "Calibration complete! The node will now exit.");

        calibration_complete_ = true;
        
        // 延迟后退出程序
        rclcpp::sleep_for(std::chrono::seconds(2));
        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::vector<std::array<double, 3>> accel_buffer_;
    double sum_sin_yaw_ = 0.0;
    double sum_cos_yaw_ = 0.0;
    int sample_count_ = 0;
    bool calibration_complete_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUGravityCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
