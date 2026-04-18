/**
 * @file sensor_monitor.cpp
 * @brief 传感器监控节点
 *
 * 功能：订阅 IMU、激光雷达（LaserScan）、里程计（Odometry）数据，
 *       实时汇总打印关键指标，便于实验观测与报告截图。
 *
 * 订阅话题：
 *   /imu         (sensor_msgs/Imu)
 *   /scan        (sensor_msgs/LaserScan)
 *   /odom        (nav_msgs/Odometry)
 *
 * 发布话题：
 *   /diagnostics_text  (std_msgs/String)  ── 供 rqt 展示
 *
 * 编译依赖：rclcpp, sensor_msgs, nav_msgs, std_msgs, geometry_msgs
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <chrono>

using namespace std::chrono_literals;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::LaserScan;
using nav_msgs::msg::Odometry;
using std_msgs::msg::String;

// ─────────────────────────────────────────────────────────────────────────────
// 工具：四元数 → 欧拉角（RPY, 单位 deg）
// ─────────────────────────────────────────────────────────────────────────────
struct EulerDeg { double roll, pitch, yaw; };

static EulerDeg quat_to_euler_deg(
  double qx, double qy, double qz, double qw)
{
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (qw * qy - qz * qx);
  double pitch = (std::abs(sinp) >= 1.0) ?
    std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  constexpr double RAD2DEG = 180.0 / M_PI;
  return {roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG};
}

// ─────────────────────────────────────────────────────────────────────────────
// SensorMonitorNode
// ─────────────────────────────────────────────────────────────────────────────
class SensorMonitorNode : public rclcpp::Node
{
public:
  SensorMonitorNode()
  : Node("sensor_monitor"),
    imu_received_(false),
    scan_received_(false),
    odom_received_(false)
  {
    // ── 订阅者 ────────────────────────────────────────────────────────────
    sub_imu_  = this->create_subscription<Imu>(
      "imu", 10,
      [this](const Imu::SharedPtr msg) { imu_callback(msg); });

    sub_scan_ = this->create_subscription<LaserScan>(
      "scan", 10,
      [this](const LaserScan::SharedPtr msg) { scan_callback(msg); });

    sub_odom_ = this->create_subscription<Odometry>(
      "odom", 10,
      [this](const Odometry::SharedPtr msg) { odom_callback(msg); });

    // ── 发布者（供 rqt Topic Monitor 查看） ──────────────────────────────
    pub_diag_ = this->create_publisher<String>("diagnostics_text", 10);

    // ── 定时打印 / 发布（1 Hz） ───────────────────────────────────────────
    timer_ = this->create_wall_timer(
      1s, [this]() { report(); });

    RCLCPP_INFO(get_logger(), "传感器监控节点已启动。");
    RCLCPP_INFO(get_logger(), "订阅: /imu  /scan  /odom");
    RCLCPP_INFO(get_logger(), "发布: /diagnostics_text");
  }

private:
  // ── IMU 回调 ──────────────────────────────────────────────────────────────
  void imu_callback(const Imu::SharedPtr msg)
  {
    imu_received_ = true;

    // 角速度（rad/s）
    imu_gyro_x_ = msg->angular_velocity.x;
    imu_gyro_y_ = msg->angular_velocity.y;
    imu_gyro_z_ = msg->angular_velocity.z;

    // 线加速度（m/s²）
    imu_acc_x_ = msg->linear_acceleration.x;
    imu_acc_y_ = msg->linear_acceleration.y;
    imu_acc_z_ = msg->linear_acceleration.z;

    // 姿态（四元数 → 欧拉角）
    auto e = quat_to_euler_deg(
      msg->orientation.x, msg->orientation.y,
      msg->orientation.z, msg->orientation.w);
    imu_roll_  = e.roll;
    imu_pitch_ = e.pitch;
    imu_yaw_   = e.yaw;

    // 合加速度（排除重力参考）
    imu_acc_norm_ = std::sqrt(
      imu_acc_x_ * imu_acc_x_ +
      imu_acc_y_ * imu_acc_y_ +
      imu_acc_z_ * imu_acc_z_);
  }

  // ── LaserScan 回调 ────────────────────────────────────────────────────────
  void scan_callback(const LaserScan::SharedPtr msg)
  {
    scan_received_  = true;
    scan_num_rays_  = static_cast<int>(msg->ranges.size());
    scan_range_min_ = msg->range_min;
    scan_range_max_ = msg->range_max;

    // 过滤无效点后统计
    std::vector<float> valid_ranges;
    valid_ranges.reserve(scan_num_rays_);
    for (auto r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        valid_ranges.push_back(r);
      }
    }

    if (!valid_ranges.empty()) {
      scan_min_dist_ = *std::min_element(valid_ranges.begin(), valid_ranges.end());
      scan_max_dist_ = *std::max_element(valid_ranges.begin(), valid_ranges.end());
      double sum     = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0f);
      scan_avg_dist_ = static_cast<double>(sum) / valid_ranges.size();
      scan_valid_    = static_cast<int>(valid_ranges.size());
    } else {
      scan_min_dist_ = scan_max_dist_ = scan_avg_dist_ = 0.0;
      scan_valid_ = 0;
    }
  }

  // ── Odometry 回调 ─────────────────────────────────────────────────────────
  void odom_callback(const Odometry::SharedPtr msg)
  {
    odom_received_ = true;
    odom_x_  = msg->pose.pose.position.x;
    odom_y_  = msg->pose.pose.position.y;

    auto e = quat_to_euler_deg(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    odom_yaw_ = e.yaw;

    odom_vel_linear_  = msg->twist.twist.linear.x;
    odom_vel_angular_ = msg->twist.twist.angular.z;
  }

  // ── 周期性报告 ────────────────────────────────────────────────────────────
  void report()
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);

    oss << "======= 传感器监控报告 =======\n";

    // IMU
    if (imu_received_) {
      oss << "[IMU]\n"
          << "  姿态 (roll/pitch/yaw): "
          << imu_roll_ << "° / " << imu_pitch_ << "° / " << imu_yaw_ << "°\n"
          << "  角速度 (xyz rad/s):    "
          << imu_gyro_x_ << " / " << imu_gyro_y_ << " / " << imu_gyro_z_ << "\n"
          << "  线加速度 (xyz m/s²):   "
          << imu_acc_x_ << " / " << imu_acc_y_ << " / " << imu_acc_z_ << "\n"
          << "  合加速度:              " << imu_acc_norm_ << " m/s²\n";
    } else {
      oss << "[IMU] 尚未接收到数据...\n";
    }

    // LaserScan
    if (scan_received_) {
      oss << "[激光雷达]\n"
          << "  射线数:   " << scan_num_rays_ << "  有效: " << scan_valid_ << "\n"
          << "  测距范围: [" << scan_range_min_ << ", " << scan_range_max_ << "] m\n"
          << "  最近障碍: " << scan_min_dist_ << " m\n"
          << "  最远测点: " << scan_max_dist_ << " m\n"
          << "  平均距离: " << scan_avg_dist_ << " m\n";
    } else {
      oss << "[激光雷达] 尚未接收到数据...\n";
    }

    // Odometry
    if (odom_received_) {
      oss << "[里程计]\n"
          << "  位置 (x/y):    " << odom_x_ << " m / " << odom_y_ << " m\n"
          << "  偏航角:        " << odom_yaw_ << "°\n"
          << "  线速度:        " << odom_vel_linear_  << " m/s\n"
          << "  角速度:        " << odom_vel_angular_ << " rad/s\n";
    } else {
      oss << "[里程计] 尚未接收到数据...\n";
    }

    oss << "==============================";

    std::string report_str = oss.str();

    // 控制台打印（每秒一次）
    RCLCPP_INFO(get_logger(), "\n%s", report_str.c_str());

    // 发布为 String 话题（供 rqt 的 Topic Monitor 插件显示）
    auto pub_msg = std_msgs::msg::String();
    pub_msg.data = report_str;
    pub_diag_->publish(pub_msg);
  }

  // ── 成员变量：订阅者 & 发布者 & 定时器 ───────────────────────────────────
  rclcpp::Subscription<Imu>::SharedPtr       sub_imu_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<Odometry>::SharedPtr  sub_odom_;
  rclcpp::Publisher<String>::SharedPtr       pub_diag_;
  rclcpp::TimerBase::SharedPtr               timer_;

  // ── IMU 数据缓存 ──────────────────────────────────────────────────────────
  bool   imu_received_;
  double imu_roll_{0}, imu_pitch_{0}, imu_yaw_{0};
  double imu_gyro_x_{0}, imu_gyro_y_{0}, imu_gyro_z_{0};
  double imu_acc_x_{0}, imu_acc_y_{0}, imu_acc_z_{0};
  double imu_acc_norm_{0};

  // ── 激光雷达数据缓存 ──────────────────────────────────────────────────────
  bool  scan_received_;
  int   scan_num_rays_{0};
  int   scan_valid_{0};
  float scan_range_min_{0}, scan_range_max_{0};
  double scan_min_dist_{0}, scan_max_dist_{0}, scan_avg_dist_{0};

  // ── 里程计数据缓存 ────────────────────────────────────────────────────────
  bool   odom_received_;
  double odom_x_{0}, odom_y_{0}, odom_yaw_{0};
  double odom_vel_linear_{0}, odom_vel_angular_{0};
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
