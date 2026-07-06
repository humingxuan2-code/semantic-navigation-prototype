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

// 与目标点距离小于 2.5 cm 时，视为到达。
constexpr double kGoalToleranceM = 0.025;

// 朝向误差小于约 2 度时，允许前进。
constexpr double kHeadingToleranceRad = 0.035;

// 距离闭环参数。
constexpr double kMaxLinearSpeed = 0.18;
constexpr double kMinLinearSpeed = 0.025;
constexpr double kDistanceGain = 0.45;

// 朝向闭环参数。
constexpr double kMaxAngularSpeed = 0.80;
constexpr double kMinAngularSpeed = 0.08;
constexpr double kYawGain = 1.50;

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

      this->x_ = msg.pose().position().x();
      this->y_ = msg.pose().position().y();
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

  bool GetPose(double &out_x, double &out_y, double &out_yaw) const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->received_)
      return false;

    out_x = this->x_;
    out_y = this->y_;
    out_yaw = this->yaw_;
    return true;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;

  bool received_{false};
  double x_{0.0};
  double y_{0.0};
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

template <typename Publisher>
void PublishFor(
    Publisher &publisher,
    double linear_x,
    double angular_z,
    std::chrono::milliseconds duration)
{
  ignition::msgs::Twist msg;
  SetTwist(msg, linear_x, angular_z);

  const auto deadline =
      std::chrono::steady_clock::now() + duration;

  while (std::chrono::steady_clock::now() < deadline)
  {
    publisher.Publish(msg);
    std::this_thread::sleep_for(50ms);
  }
}

template <typename Publisher>
void StopRobot(Publisher &publisher)
{
  // 连续发送 2 秒零速度，避免保留前一次运动状态。
  PublishFor(publisher, 0.0, 0.0, 2000ms);
}

struct TargetRequest
{
  bool relative{false};
  double first{0.0};
  double second{0.0};
};

bool ParseRequest(
    int argc,
    char **argv,
    TargetRequest &request)
{
  if (argc != 4)
  {
    std::cerr << "用法：" << std::endl;
    std::cerr << "  " << argv[0]
              << " abs <目标x> <目标y>" << std::endl;
    std::cerr << "  " << argv[0]
              << " rel <前进距离m> <左移距离m>" << std::endl;

    return false;
  }

  try
  {
    const std::string mode = argv[1];

    request.first = std::stod(argv[2]);
    request.second = std::stod(argv[3]);

    if (mode == "abs")
    {
      request.relative = false;
      return true;
    }

    if (mode == "rel")
    {
      request.relative = true;
      return true;
    }

    std::cerr << "模式只能是 abs 或 rel。" << std::endl;
    return false;
  }
  catch (const std::exception &)
  {
    std::cerr << "坐标参数必须是数字。" << std::endl;
    return false;
  }
}
}  // namespace

int main(int argc, char **argv)
{
  TargetRequest request;

  if (!ParseRequest(argc, argv, request))
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
    std::cerr << "无法订阅 odometry: "
              << kOdomTopic << std::endl;
    return 1;
  }

  auto publisher =
      node.Advertise<ignition::msgs::Twist>(kCmdTopic);

  if (!publisher)
  {
    std::cerr << "无法创建速度发布器: "
              << kCmdTopic << std::endl;
    return 1;
  }

  std::cout << "等待蓝车 odometry..." << std::endl;

  if (!odom.WaitForFirstMessage(5000ms))
  {
    std::cerr << "5 秒内没有收到 odometry，请确认 Gazebo 正在运行。"
              << std::endl;
    return 1;
  }

  // 等待 Gazebo DiffDrive 控制器发现 publisher。
  std::this_thread::sleep_for(800ms);

  // 运行前先稳定发送一小段停止指令。
  PublishFor(publisher, 0.0, 0.0, 500ms);

  double start_x = 0.0;
  double start_y = 0.0;
  double start_yaw = 0.0;

  odom.GetPose(start_x, start_y, start_yaw);

  double target_x = 0.0;
  double target_y = 0.0;

  if (request.relative)
  {
    // rel <前进距离> <左移距离>
    const double forward_m = request.first;
    const double left_m = request.second;

    target_x =
        start_x
        + forward_m * std::cos(start_yaw)
        - left_m * std::sin(start_yaw);

    target_y =
        start_y
        + forward_m * std::sin(start_yaw)
        + left_m * std::cos(start_yaw);
  }
  else
  {
    // abs <世界坐标x> <世界坐标y>
    target_x = request.first;
    target_y = request.second;
  }

  const double initial_distance =
      std::hypot(target_x - start_x, target_y - start_y);

  std::cout << std::fixed << std::setprecision(3);

  std::cout << "起点: x=" << start_x
            << ", y=" << start_y
            << ", yaw=" << RadToDeg(start_yaw)
            << " 度" << std::endl;

  std::cout << "目标点: x=" << target_x
            << ", y=" << target_y << std::endl;

  std::cout << "初始目标距离: "
            << initial_distance
            << " m" << std::endl;

  const auto timeout =
      std::chrono::steady_clock::now() + 30s;

  auto next_report =
      std::chrono::steady_clock::now();

  bool reached_goal = false;

  while (std::chrono::steady_clock::now() < timeout)
  {
    double current_x = 0.0;
    double current_y = 0.0;
    double current_yaw = 0.0;

    if (!odom.GetPose(current_x, current_y, current_yaw))
      continue;

    const double dx = target_x - current_x;
    const double dy = target_y - current_y;

    const double distance = std::hypot(dx, dy);

    if (distance <= kGoalToleranceM)
    {
      reached_goal = true;

      std::cout << "已进入目标点距离阈值，开始停车。"
                << std::endl;

      break;
    }

    const double target_heading = std::atan2(dy, dx);

    const double heading_error =
        NormalizeAngle(target_heading - current_yaw);

    double linear_speed = 0.0;
    double angular_speed = 0.0;
    std::string mode;

    if (std::abs(heading_error) > kHeadingToleranceRad)
    {
      // 朝向不对时，只转向，不前进。
      mode = "转向";

      const double speed =
          std::clamp(
              kYawGain * std::abs(heading_error),
              kMinAngularSpeed,
              kMaxAngularSpeed);

      angular_speed =
          (heading_error >= 0.0)
              ? speed
              : -speed;
    }
    else
    {
      // 朝向对准后，按距离比例减速前进。
      mode = "前进";

      linear_speed =
          std::clamp(
              kDistanceGain * distance,
              kMinLinearSpeed,
              kMaxLinearSpeed);
    }

    ignition::msgs::Twist command;
    SetTwist(command, linear_speed, angular_speed);

    publisher.Publish(command);

    if (std::chrono::steady_clock::now() >= next_report)
    {
      std::cout << "模式: " << mode
                << " | 当前: ("
                << current_x << ", "
                << current_y << ")"
                << " | 剩余距离: "
                << distance
                << " m"
                << " | 朝向误差: "
                << RadToDeg(heading_error)
                << " 度"
                << " | v="
                << linear_speed
                << " m/s"
                << " | w="
                << angular_speed
                << " rad/s"
                << std::endl;

      next_report += 300ms;
    }

    std::this_thread::sleep_for(50ms);
  }

  StopRobot(publisher);

  double final_x = 0.0;
  double final_y = 0.0;
  double final_yaw = 0.0;

  odom.GetPose(final_x, final_y, final_yaw);

  const double final_error =
      std::hypot(target_x - final_x, target_y - final_y);

  if (!reached_goal)
  {
    std::cerr << "警告：30 秒内未到达目标点，已安全停止。"
              << std::endl;

    std::cerr << "最终目标误差: "
              << final_error
              << " m" << std::endl;

    return 2;
  }

  std::cout << "完成。" << std::endl;

  std::cout << "终点: x=" << final_x
            << ", y=" << final_y
            << ", yaw=" << RadToDeg(final_yaw)
            << " 度" << std::endl;

  std::cout << "最终目标误差: "
            << final_error
            << " m" << std::endl;

  return 0;
}
