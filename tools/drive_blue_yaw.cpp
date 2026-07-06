#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
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

constexpr double kPi = 3.14159265358979323846;

// 距离目标角度约 1.15 度以内时停止。
constexpr double kStopMarginRad = 0.020;

// 闭环角速度控制参数。
constexpr double kMaxAngularSpeed = 0.80;  // rad/s
constexpr double kMinAngularSpeed = 0.08;  // rad/s
constexpr double kYawGain = 1.50;

// 把角度归一化到 (-π, π]。
double NormalizeAngle(double angle)
{
  while (angle > kPi)
    angle -= 2.0 * kPi;

  while (angle <= -kPi)
    angle += 2.0 * kPi;

  return angle;
}

double RadToDeg(double radians)
{
  return radians * 180.0 / kPi;
}

// 从四元数中计算平面朝向 yaw。
double YawFromQuaternion(const ignition::msgs::Quaternion &q)
{
  const double siny_cosp =
      2.0 * (q.w() * q.z() + q.x() * q.y());

  const double cosy_cosp =
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());

  return std::atan2(siny_cosp, cosy_cosp);
}

class OdomListener
{
public:
  void OnOdom(const ignition::msgs::Odometry &msg)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);

      this->yaw_ = YawFromQuaternion(msg.pose().orientation());
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

  bool GetYaw(double &out_yaw) const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->received_)
      return false;

    out_yaw = this->yaw_;
    return true;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;

  bool received_{false};
  double yaw_{0.0};
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

// 连续发送零速度 2 秒，避免机器人残留上一次的速度。
template <typename Publisher>
void StopRobot(Publisher &publisher)
{
  ignition::msgs::Twist stop_msg;
  SetTwist(stop_msg, 0.0, 0.0);

  const auto deadline =
      std::chrono::steady_clock::now() + 2000ms;

  while (std::chrono::steady_clock::now() < deadline)
  {
    publisher.Publish(stop_msg);
    std::this_thread::sleep_for(50ms);
  }
}

bool ParseAction(
    int argc,
    char **argv,
    std::string &action,
    double &target_delta)
{
  if (argc < 2)
  {
    std::cerr
        << "用法: " << argv[0]
        << " {left90|right90}"
        << std::endl;

    return false;
  }

  action = argv[1];

  if (action == "left90")
  {
    target_delta = kPi / 2.0;
    return true;
  }

  if (action == "right90")
  {
    target_delta = -kPi / 2.0;
    return true;
  }

  std::cerr
      << "未知动作: " << action
      << std::endl;

  std::cerr
      << "用法: " << argv[0]
      << " {left90|right90}"
      << std::endl;

  return false;
}
}  // namespace

int main(int argc, char **argv)
{
  std::string action;
  double target_delta = 0.0;

  if (!ParseAction(argc, argv, action, target_delta))
    return 1;

  ignition::transport::Node node;
  OdomListener odom;

  const bool subscribed =
      node.Subscribe(
          kOdomTopic,
          &OdomListener::OnOdom,
          &odom);

  if (!subscribed)
  {
    std::cerr
        << "无法订阅 odometry: "
        << kOdomTopic
        << std::endl;

    return 1;
  }

  auto publisher =
      node.Advertise<ignition::msgs::Twist>(kCmdTopic);

  if (!publisher)
  {
    std::cerr
        << "无法创建速度发布器: "
        << kCmdTopic
        << std::endl;

    return 1;
  }

  std::cout << "等待蓝车 odometry..." << std::endl;

  if (!odom.WaitForFirstMessage(5000ms))
  {
    std::cerr
        << "5 秒内没有收到 odometry，请确认 Gazebo 正在运行。"
        << std::endl;

    return 1;
  }

  // 给 Gazebo DiffDrive 控制插件一点时间发现 publisher。
  std::this_thread::sleep_for(800ms);

  double start_yaw = 0.0;
  odom.GetYaw(start_yaw);

  const double target_yaw =
      NormalizeAngle(start_yaw + target_delta);

  std::cout << std::fixed << std::setprecision(2);

  std::cout
      << "起始 yaw: "
      << RadToDeg(start_yaw)
      << " 度"
      << std::endl;

  std::cout
      << "目标 yaw: "
      << RadToDeg(target_yaw)
      << " 度"
      << std::endl;

  std::cout
      << "动作: "
      << ((target_delta > 0.0)
              ? "左转 90 度"
              : "右转 90 度")
      << std::endl;

  const auto timeout =
      std::chrono::steady_clock::now() + 15s;

  auto next_report = std::chrono::steady_clock::now();

  bool reached_goal = false;

  while (std::chrono::steady_clock::now() < timeout)
  {
    double current_yaw = 0.0;

    if (!odom.GetYaw(current_yaw))
      continue;

    const double yaw_error =
        NormalizeAngle(target_yaw - current_yaw);

    if (std::abs(yaw_error) <= kStopMarginRad)
    {
      reached_goal = true;

      std::cout
          << "已进入角度停止阈值，开始停车。"
          << std::endl;

      break;
    }

    const double angular_speed =
        std::clamp(
            kYawGain * std::abs(yaw_error),
            kMinAngularSpeed,
            kMaxAngularSpeed);

    const double signed_angular_speed =
        (yaw_error >= 0.0)
            ? angular_speed
            : -angular_speed;

    ignition::msgs::Twist command;
    SetTwist(command, 0.0, signed_angular_speed);

    publisher.Publish(command);

    if (std::chrono::steady_clock::now() >= next_report)
    {
      std::cout
          << "当前 yaw: "
          << RadToDeg(current_yaw)
          << " 度，误差: "
          << RadToDeg(yaw_error)
          << " 度，角速度: "
          << signed_angular_speed
          << " rad/s"
          << std::endl;

      next_report += 300ms;
    }

    std::this_thread::sleep_for(50ms);
  }

  StopRobot(publisher);

  double final_yaw = 0.0;
  odom.GetYaw(final_yaw);

  const double final_error =
      NormalizeAngle(target_yaw - final_yaw);

  const double actual_turn =
      NormalizeAngle(final_yaw - start_yaw);

  if (!reached_goal)
  {
    std::cerr
        << "警告：15 秒内未达到目标角度，已安全停止。"
        << std::endl;

    return 2;
  }

  std::cout << "完成。" << std::endl;

  std::cout
      << "最终 yaw: "
      << RadToDeg(final_yaw)
      << " 度"
      << std::endl;

  std::cout
      << "实际转角: "
      << RadToDeg(actual_turn)
      << " 度"
      << std::endl;

  std::cout
      << "最终角度误差: "
      << std::abs(RadToDeg(final_error))
      << " 度"
      << std::endl;

  return 0;
}
