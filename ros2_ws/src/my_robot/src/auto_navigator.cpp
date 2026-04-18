/**
 * @file auto_navigator.cpp
 * @brief 自动导航避障演示节点
 *
 * 功能：机器人在仿真环境中自主巡航。
 *   1. 正常行驶时匀速前进；
 *   2. 前方检测到障碍物（< STOP_DIST）时原地停止；
 *   3. 判断左右哪侧空间更大，向宽阔侧旋转后继续前行；
 *   4. 通过参数配置速度、距离阈值；
 *   5. 将当前状态发布为 /robot_state（std_msgs/String）供 rqt 监控。
 *
 * 订阅：/scan (sensor_msgs/LaserScan)
 * 发布：/cmd_vel (geometry_msgs/Twist)
 *        /robot_state (std_msgs/String)
 *
 * 状态机：FORWARD → STOP → ROTATE → FORWARD → ...
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;
using sensor_msgs::msg::LaserScan;
using geometry_msgs::msg::Twist;
using std_msgs::msg::String;

// ─────────────────────────────────────────────────────────────────────────────
// 状态机枚举
// ─────────────────────────────────────────────────────────────────────────────
enum class State {
  FORWARD,  ///< 直行前进
  STOP,     ///< 检测到障碍，停止判断
  ROTATE    ///< 旋转避障
};

static const char * state_name(State s)
{
  switch (s) {
    case State::FORWARD: return "FORWARD（直行）";
    case State::STOP:    return "STOP（停止判断）";
    case State::ROTATE:  return "ROTATE（旋转避障）";
    default:             return "UNKNOWN";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// AutoNavigatorNode
// ─────────────────────────────────────────────────────────────────────────────
class AutoNavigatorNode : public rclcpp::Node
{
public:
  AutoNavigatorNode()
  : Node("auto_navigator"),
    state_(State::FORWARD),
    rotate_direction_(1.0),
    rotate_elapsed_(0.0)
  {
    // ── 参数声明 ──────────────────────────────────────────────────────────
    this->declare_parameter<double>("linear_speed",   0.3);    // m/s
    this->declare_parameter<double>("angular_speed",  0.8);    // rad/s
    this->declare_parameter<double>("stop_dist",      0.55);   // m，停止距离
    this->declare_parameter<double>("warn_dist",      0.80);   // m，减速距离
    this->declare_parameter<double>("front_angle_deg", 45.0);  // °，前方检测角
    this->declare_parameter<double>("rotate_time",    2.0);    // s，最小旋转时长

    linear_speed_  = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    stop_dist_     = this->get_parameter("stop_dist").as_double();
    warn_dist_     = this->get_parameter("warn_dist").as_double();
    front_angle_   = this->get_parameter("front_angle_deg").as_double()
                     * M_PI / 180.0;    // 转 rad
    rotate_time_   = this->get_parameter("rotate_time").as_double();

    // ── 话题 ──────────────────────────────────────────────────────────────
    sub_scan_ = this->create_subscription<LaserScan>(
      "scan", 10,
      [this](const LaserScan::SharedPtr msg) { scan_callback(msg); });

    pub_cmd_   = this->create_publisher<Twist>("cmd_vel", 10);
    pub_state_ = this->create_publisher<String>("robot_state", 10);

    // ── 控制定时器（20 Hz） ──────────────────────────────────────────────
    constexpr double CTRL_HZ = 20.0;
    dt_ = 1.0 / CTRL_HZ;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
      [this]() { control_loop(); });

    RCLCPP_INFO(get_logger(), "自动导航节点已启动。");
    RCLCPP_INFO(get_logger(),
      "参数: 前进速%.2f m/s | 旋转速%.2f rad/s | "
      "停止距%.2f m | 减速距%.2f m | 旋转时%.2f s",
      linear_speed_, angular_speed_, stop_dist_, warn_dist_, rotate_time_);
  }

private:
  // ── LaserScan 回调 ────────────────────────────────────────────────────────
  void scan_callback(const LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // 从扫描数据中提取指定角度范围的最小距离
  //   angle_min_rad / angle_max_rad：相对激光坐标系，0 = 正前方
  // ─────────────────────────────────────────────────────────────────────────
  double min_range_in_sector(
    const LaserScan::SharedPtr & scan,
    double angle_min_rad, double angle_max_rad) const
  {
    if (!scan) {
      return std::numeric_limits<double>::infinity();
    }
    double min_r = std::numeric_limits<double>::infinity();
    int n = static_cast<int>(scan->ranges.size());
    for (int i = 0; i < n; ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      if (angle >= angle_min_rad && angle <= angle_max_rad) {
        float r = scan->ranges[i];
        if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
          min_r = std::min(min_r, static_cast<double>(r));
        }
      }
    }
    return min_r;
  }

  // 统计扇形区域内有效点平均距离（用于判断左右空间大小）
  double mean_range_in_sector(
    const LaserScan::SharedPtr & scan,
    double angle_min_rad, double angle_max_rad) const
  {
    if (!scan) {
      return 0.0;
    }
    double sum = 0.0;
    int cnt = 0;
    int n = static_cast<int>(scan->ranges.size());
    for (int i = 0; i < n; ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      if (angle >= angle_min_rad && angle <= angle_max_rad) {
        float r = scan->ranges[i];
        if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
          sum += r;
          cnt++;
        }
      }
    }
    return (cnt > 0) ? (sum / cnt) : 0.0;
  }

  // ── 主控制循环（20 Hz） ───────────────────────────────────────────────────
  void control_loop()
  {
    Twist cmd;

    // ── 从激光雷达提取关键距离 ─────────────────────────────────────────────
    double front_min = min_range_in_sector(
      latest_scan_, -front_angle_, front_angle_);

    // 前方 0~warn_dist 内是否有障碍
    bool obstacle_near = (front_min < stop_dist_);
    bool obstacle_warn = (front_min < warn_dist_);

    // 左侧（正角方向）和右侧（负角方向）平均距离，用于选转向
    double left_space  = mean_range_in_sector(
      latest_scan_, front_angle_, M_PI);
    double right_space = mean_range_in_sector(
      latest_scan_, -M_PI, -front_angle_);

    // ── 状态机转换 ────────────────────────────────────────────────────────
    switch (state_) {

      case State::FORWARD:
        if (obstacle_near) {
          // 切换到停止判断态
          state_ = State::STOP;
          RCLCPP_WARN(get_logger(),
            "前方障碍距离 %.2f m (<%.2f m)，进入 STOP 状态", front_min, stop_dist_);
          cmd.linear.x  = 0.0;
          cmd.angular.z = 0.0;
        } else if (obstacle_warn) {
          // 减速通过
          double ratio = (front_min - stop_dist_) / (warn_dist_ - stop_dist_);
          cmd.linear.x  = linear_speed_ * std::max(0.2, ratio);
          cmd.angular.z = 0.0;
        } else {
          // 全速前进
          cmd.linear.x  = linear_speed_;
          cmd.angular.z = 0.0;
        }
        break;

      case State::STOP:
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        // 决定旋转方向：哪侧空间大就往哪转
        rotate_direction_ = (left_space >= right_space) ? 1.0 : -1.0;
        rotate_elapsed_   = 0.0;
        state_ = State::ROTATE;
        RCLCPP_INFO(get_logger(),
          "左侧空间 %.2f m，右侧空间 %.2f m，%s旋转",
          left_space, right_space,
          (rotate_direction_ > 0) ? "左（正）" : "右（负）");
        break;

      case State::ROTATE:
        rotate_elapsed_ += dt_;
        cmd.linear.x  = 0.0;
        cmd.angular.z = rotate_direction_ * angular_speed_;

        // 旋转足够时间 且 前方已无障碍
        if (rotate_elapsed_ >= rotate_time_ && front_min > warn_dist_) {
          state_ = State::FORWARD;
          RCLCPP_INFO(get_logger(),
            "旋转 %.1f s，前方已清空（%.2f m），恢复 FORWARD",
            rotate_elapsed_, front_min);
          cmd.angular.z = 0.0;
        }
        break;
    }

    // ── 发布速度指令 ──────────────────────────────────────────────────────
    pub_cmd_->publish(cmd);

    // ── 发布状态字符串（1 Hz：每 20 次控制循环打印一次）──────────────────
    if (++report_cnt_ >= 20) {
      report_cnt_ = 0;
      String state_msg;
      std::ostringstream oss;
      oss << "[auto_navigator] 状态: " << state_name(state_)
          << "  前方: " << std::setprecision(2) << front_min << " m"
          << "  cmd_vel: lin=" << cmd.linear.x
          << " ang=" << cmd.angular.z;
      state_msg.data = oss.str();
      pub_state_->publish(state_msg);
      RCLCPP_INFO(get_logger(), "%s", state_msg.data.c_str());
    }
  }

  // ── 成员变量 ──────────────────────────────────────────────────────────────
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<Twist>::SharedPtr        pub_cmd_;
  rclcpp::Publisher<String>::SharedPtr       pub_state_;
  rclcpp::TimerBase::SharedPtr               timer_;

  LaserScan::SharedPtr latest_scan_;

  // 参数
  double linear_speed_;
  double angular_speed_;
  double stop_dist_;
  double warn_dist_;
  double front_angle_;    // rad
  double rotate_time_;    // s
  double dt_;             // 控制周期

  // 状态机
  State  state_;
  double rotate_direction_;
  double rotate_elapsed_;

  // 报告计数器
  int report_cnt_{0};

  // 用于 setprecision（需包含 <iomanip>）
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoNavigatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
