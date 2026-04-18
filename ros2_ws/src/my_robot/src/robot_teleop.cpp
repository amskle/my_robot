/**
 * @file robot_teleop.cpp
 * @brief 键盘遥控节点
 *
 * 功能：读取键盘输入，将按键映射为线速度 / 角速度指令，
 *       发布到 /cmd_vel（geometry_msgs/Twist）控制差速机器人。
 *
 * 按键映射：
 *   w / ↑  ── 前进          s / ↓  ── 后退
 *   a / ←  ── 左转          d / →  ── 右转
 *   q       ── 加速（线速）  e       ── 减速（线速）
 *   z       ── 加速（角速）  c       ── 减速（角速）
 *   空格    ── 紧急停止      Ctrl+C  ── 退出
 *
 * 编译依赖：rclcpp, geometry_msgs
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// POSIX 终端原始模式
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstdio>
#include <cstring>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <map>
#include <string>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// 终端工具：切换原始/恢复模式
// ─────────────────────────────────────────────────────────────────────────────
namespace terminal
{

struct termios original_attrs;

void set_raw_mode()
{
  struct termios raw;
  tcgetattr(STDIN_FILENO, &original_attrs);
  raw = original_attrs;
  raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);  // 关闭行缓冲与回显
  raw.c_cc[VMIN]  = 0;   // 非阻塞读取
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void restore()
{
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_attrs);
}

/**
 * @brief 读取一个字符（非阻塞）。返回 -1 表示无输入。
 *        处理方向键的 ESC 序列（ESC [ A/B/C/D）。
 */
int read_key()
{
  unsigned char c = 0;
  if (read(STDIN_FILENO, &c, 1) != 1) {
    return -1;
  }
  // ESC 序列（方向键）
  if (c == 0x1B) {
    unsigned char seq[2] = {0, 0};
    if (read(STDIN_FILENO, &seq[0], 1) == 1 &&
        read(STDIN_FILENO, &seq[1], 1) == 1 &&
        seq[0] == '[') {
      switch (seq[1]) {
        case 'A': return 'w';  // ↑
        case 'B': return 's';  // ↓
        case 'C': return 'd';  // →
        case 'D': return 'a';  // ←
        default: break;
      }
    }
    return -1;
  }
  return static_cast<int>(c);
}

}  // namespace terminal

// ─────────────────────────────────────────────────────────────────────────────
// RobotTeleopNode
// ─────────────────────────────────────────────────────────────────────────────
class RobotTeleopNode : public rclcpp::Node
{
public:
  RobotTeleopNode()
  : Node("robot_teleop"),
    linear_vel_(0.0), angular_vel_(0.0),
    linear_step_(0.05), angular_step_(0.1),
    max_linear_(1.0),  max_angular_(2.0),
    running_(true)
  {
    // 声明 & 获取参数
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<double>("linear_step",  0.05);
    this->declare_parameter<double>("angular_step", 0.10);
    this->declare_parameter<double>("max_linear",   1.0);
    this->declare_parameter<double>("max_angular",  2.0);

    auto topic    = this->get_parameter("cmd_vel_topic").as_string();
    linear_step_  = this->get_parameter("linear_step").as_double();
    angular_step_ = this->get_parameter("angular_step").as_double();
    max_linear_   = this->get_parameter("max_linear").as_double();
    max_angular_  = this->get_parameter("max_angular").as_double();

    // 创建发布者
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

    // 定时器：以 10 Hz 持续发布当前速度（避免 cmd_vel_timeout 触发停止）
    timer_ = this->create_wall_timer(
      100ms,
      [this]() { this->publish_cmd(); });

    terminal::set_raw_mode();
    print_help();

    // 后台线程持续读取键盘
    key_thread_ = std::thread([this]() { this->key_loop(); });
  }

  ~RobotTeleopNode()
  {
    running_ = false;
    if (key_thread_.joinable()) {
      key_thread_.join();
    }
    stop_robot();
    terminal::restore();
    RCLCPP_INFO(get_logger(), "遥控节点已退出，终端已恢复。");
  }

private:
  // ── 发布当前速度指令 ──────────────────────────────────────────────────────
  void publish_cmd()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear_vel_;
    msg.angular.z = angular_vel_;
    pub_->publish(msg);
  }

  // ── 发送停车指令 ──────────────────────────────────────────────────────────
  void stop_robot()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = 0.0;
    msg.angular.z = 0.0;
    pub_->publish(msg);
  }

  // ── 速度裁剪辅助 ─────────────────────────────────────────────────────────
  static double clamp(double v, double lo, double hi)
  {
    return (v < lo) ? lo : (v > hi ? hi : v);
  }

  // ── 键盘读取循环（后台线程） ──────────────────────────────────────────────
  void key_loop()
  {
    while (running_ && rclcpp::ok()) {
      int key = terminal::read_key();
      if (key == -1) {
        std::this_thread::sleep_for(10ms);
        continue;
      }
      handle_key(key);
    }
  }

  // ── 按键处理逻辑 ──────────────────────────────────────────────────────────
  void handle_key(int key)
  {
    // 打印当前按键（调试）
    // RCLCPP_DEBUG(get_logger(), "key: %d ('%c')", key, key);

    switch (key) {
      // 前进
      case 'w': case 'W':
        linear_vel_ += linear_step_;
        linear_vel_  = clamp(linear_vel_, -max_linear_, max_linear_);
        angular_vel_ = 0.0;
        break;
      // 后退
      case 's': case 'S':
        linear_vel_ -= linear_step_;
        linear_vel_  = clamp(linear_vel_, -max_linear_, max_linear_);
        angular_vel_ = 0.0;
        break;
      // 左转
      case 'a': case 'A':
        angular_vel_ += angular_step_;
        angular_vel_  = clamp(angular_vel_, -max_angular_, max_angular_);
        break;
      // 右转
      case 'd': case 'D':
        angular_vel_ -= angular_step_;
        angular_vel_  = clamp(angular_vel_, -max_angular_, max_angular_);
        break;
      // 加大线速
      case 'q': case 'Q':
        linear_step_ = std::min(linear_step_ + 0.01, 0.5);
        break;
      // 减小线速
      case 'e': case 'E':
        linear_step_ = std::max(linear_step_ - 0.01, 0.01);
        break;
      // 加大角速
      case 'z': case 'Z':
        angular_step_ = std::min(angular_step_ + 0.05, 1.0);
        break;
      // 减小角速
      case 'c': case 'C':
        angular_step_ = std::max(angular_step_ - 0.05, 0.05);
        break;
      // 停止
      case ' ':
        linear_vel_  = 0.0;
        angular_vel_ = 0.0;
        break;
      default:
        break;
    }

    // 实时打印状态
    printf("\r线速度: %+.3f m/s  角速度: %+.3f rad/s  "
           "[步长: linear=%.2f  angular=%.2f]    ",
           linear_vel_, angular_vel_, linear_step_, angular_step_);
    fflush(stdout);
  }

  // ── 打印帮助信息 ──────────────────────────────────────────────────────────
  static void print_help()
  {
    printf("\n");
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║      机器人键盘遥控节点  (robot_teleop)      ║\n");
    printf("╠══════════════════════════════════════════════╣\n");
    printf("║  w/↑  前进     s/↓  后退                    ║\n");
    printf("║  a/←  左转     d/→  右转                    ║\n");
    printf("║  q    增大线速步长  e    减小线速步长        ║\n");
    printf("║  z    增大角速步长  c    减小角速步长        ║\n");
    printf("║  空格  紧急停止     Ctrl+C  退出             ║\n");
    printf("╚══════════════════════════════════════════════╝\n");
    printf("\n");
    fflush(stdout);
  }

  // ── 成员变量 ──────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double linear_vel_;
  double angular_vel_;
  double linear_step_;
  double angular_step_;
  double max_linear_;
  double max_angular_;

  std::atomic<bool> running_;
  std::thread key_thread_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotTeleopNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
