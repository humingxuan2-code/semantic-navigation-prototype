#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

using namespace std::chrono_literals;

namespace
{
const std::string kCmdTopic = "/model/vehicle_blue/cmd_vel";
const std::string kOdomTopic = "/model/vehicle_blue/odometry";

// 距离闭环控制参数
constexpr double kMaxSpeed = 0.18;      // 离目标远时的最高线速度
constexpr double kMinSpeed = 0.025;     // 接近目标时保留的低速
constexpr double kSpeedGain = 0.45;     // 剩余距离 → 速度 的比例系数
constexpr double kStopMargin = 0.012;   // 距离目标约 1.2 cm 时开始停止

class OdomListener
{
public:
  void OnOdom(const ignition::msgs::Odometry &msg)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->x_ = msg.pose().position().x();
      this->y_ = msg.pose().position().y();
      this->received_ = true;
    }
    this->cv_.notify_all();
  }

  bool WaitForFirstMessage(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    return this->cv_.wait_for(lock, timeout, [this]() {
      return this->received_;
    });
  }

  bool GetPosition(double &out_x, double &out_y) const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->received_)
      return false;

    out_x = this->x_;
    out_y = this->y_;
    return true;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool received_{false};
  double x_{0.0};
  double y_{0.0};
};

void SetTwist(
    ignition::msgs::Twist &msg,
    double linear_x,
    double angular_z)
{
  msg.mutable_linear()->set_x(linear_x);
  msg.mutable_linear()->set_y(0.0);
  msg.mutable_linear()->set_z(0.0);

  msg.mutable_angular()->set_x(0.0);
  msg.mutable_angular()->set_y(0.0);
  msg.mutable_angular()->set_z(angular_z);
}

template <typename Publisher>
void PublishFor(
    Publisher &publisher,
    double linear_x,
    double angular_z,
    std::chrono::milliseconds duration)
{
  ignition::msgs::Twist msg;
  SetTwist(msg, linear_x, angular_z);

  const auto deadline = std::chrono::steady_clock::now() + duration;

  while (std::chrono::steady_clock::now() < deadline)
  {
    publisher.Publish(msg);
    std::this_thread::sleep_for(50ms);  // 20 Hz
  }
}

template <typename Publisher>
void StopRobot(Publisher &publisher)
{
  // 连续发送 2 秒零速度，确保不会保留之前的运动指令
  PublishFor(publisher, 0.0, 0.0, 2000ms);
}

double ParseTargetDistance(int argc, char **argv)
{
  if (argc < 2)
    return 0.50;

  try
  {
    const double value = std::stod(argv[1]);

    if (value <= 0.0 || value > 5.0)
      throw std::out_of_range("distance out of range");

    return value;
  }
  catch (const std::exception &)
  {
    std::cerr << "距离参数不合法。示例："
              << argv[0] << " 0.50" << std::endl;
    std::exit(1);
  }
}
}  // namespace

int main(int argc, char **argv)
{
  const double target_distance = ParseTargetDistance(argc, argv);

  ignition::transport::Node node;
  OdomListener odom;

  const bool subscribed = node.Subscribe(
      kOdomTopic,
      &OdomListener::OnOdom,
      &odom);

  if (!subscribed)
  {
    std::cerr << "无法订阅里程计 topic: " << kOdomTopic << std::endl;
    return 1;
  }

  auto publisher = node.Advertise<ignition::msgs::Twist>(kCmdTopic);

  if (!publisher)
  {
    std::cerr << "无法创建速度发布器: " << kCmdTopic << std::endl;
    return 1;
  }

  std::cout << "等待蓝车 odometry..." << std::endl;

  if (!odom.WaitForFirstMessage(5000ms))
  {
    std::cerr << "5 秒内没有收到 odometry。请确认 Gazebo 正在运行。"
              << std::endl;
    return 1;
  }

  double start_x = 0.0;
  double start_y = 0.0;
  odom.GetPosition(start_x, start_y);

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "起点: x=" << start_x
            << ", y=" << start_y << std::endl;
  std::cout << "目标: 前进 " << target_distance << " 米" << std::endl;

  const auto timeout = std::chrono::steady_clock::now() + 20s;
  auto next_report = std::chrono::steady_clock::now();

  bool reached_goal = false;

  while (std::chrono::steady_clock::now() < timeout)
  {
    double current_x = 0.0;
    double current_y = 0.0;
    odom.GetPosition(current_x, current_y);

    const double traveled = std::hypot(
        current_x - start_x,
        current_y - start_y);

    const double remaining = target_distance - traveled;

    if (remaining <= kStopMargin)
    {
      reached_goal = true;
      std::cout << "已进入停止阈值，开始停车。" << std::endl;
      break;
    }

    // 简单比例控制：离目标越近，速度越低
    const double speed = std::clamp(
        kSpeedGain * remaining,
        kMinSpeed,
        kMaxSpeed);

    ignition::msgs::Twist command;
    SetTwist(command, speed, 0.0);
    publisher.Publish(command);

    if (std::chrono::steady_clock::now() >= next_report)
    {
      std::cout << "当前位移: " << traveled
                << " / " << target_distance
                << " 米，剩余: " << remaining
                << " 米，速度: " << speed
                << " m/s" << std::endl;

      next_report += 300ms;
    }

    std::this_thread::sleep_for(50ms);
  }

  StopRobot(publisher);

  double final_x = 0.0;
  double final_y = 0.0;
  odom.GetPosition(final_x, final_y);

  const double final_distance = std::hypot(
      final_x - start_x,
      final_y - start_y);

  if (!reached_goal)
  {
    std::cerr << "警告：20 秒内未达到目标，已安全停止。" << std::endl;
    return 2;
  }

  std::cout << "完成。" << std::endl;
  std::cout << "终点: x=" << final_x
            << ", y=" << final_y << std::endl;
  std::cout << "实际位移: "
            << final_distance << " 米" << std::endl;
  std::cout << "绝对误差: "
            << std::abs(final_distance - target_distance)
            << " 米" << std::endl;

  return 0;
}
