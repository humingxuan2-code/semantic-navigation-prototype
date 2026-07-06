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

class OdomListener
{
public:
  void OnOdom(const ignition::msgs::Odometry &msg)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->x = msg.pose().position().x();
      this->y = msg.pose().position().y();
      this->received = true;
    }
    this->cv.notify_all();
  }

  bool WaitForFirstMessage(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    return this->cv.wait_for(lock, timeout, [this]() {
      return this->received;
    });
  }

  bool GetPosition(double &out_x, double &out_y)
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    if (!this->received)
      return false;

    out_x = this->x;
    out_y = this->y;
    return true;
  }

private:
  std::mutex mutex;
  std::condition_variable cv;
  bool received{false};
  double x{0.0};
  double y{0.0};
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
    std::this_thread::sleep_for(50ms);
  }
}

template <typename Publisher>
void StopRobot(Publisher &publisher)
{
  PublishFor(publisher, 0.0, 0.0, 2000ms);
}

double ParseDistance(int argc, char **argv)
{
  if (argc < 2)
    return 0.50;

  try
  {
    const double distance = std::stod(argv[1]);

    if (distance <= 0.0 || distance > 5.0)
      throw std::out_of_range("distance must be within (0, 5]");

    return distance;
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
  const double target_distance = ParseDistance(argc, argv);

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
    std::cerr << "5 秒内没有收到 odometry。请确认 Gazebo 正在运行。" << std::endl;
    return 1;
  }

  double start_x = 0.0;
  double start_y = 0.0;
  odom.GetPosition(start_x, start_y);

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "起点: x=" << start_x << ", y=" << start_y << std::endl;
  std::cout << "目标: 前进 " << target_distance << " 米" << std::endl;

  ignition::msgs::Twist forward_msg;
  SetTwist(forward_msg, 0.20, 0.0);

  const auto timeout = std::chrono::steady_clock::now() + 15s;
  auto next_report = std::chrono::steady_clock::now();

  bool reached_goal = false;

  while (std::chrono::steady_clock::now() < timeout)
  {
    double current_x = 0.0;
    double current_y = 0.0;

    if (!odom.GetPosition(current_x, current_y))
      continue;

    const double distance = std::hypot(
        current_x - start_x,
        current_y - start_y);

    if (std::chrono::steady_clock::now() >= next_report)
    {
      std::cout << "当前位移: "
                << distance << " / "
                << target_distance << " 米" << std::endl;
      next_report += 500ms;
    }

    if (distance >= target_distance)
    {
      reached_goal = true;
      std::cout << "达到目标距离，开始停止。" << std::endl;
      break;
    }

    publisher.Publish(forward_msg);
    std::this_thread::sleep_for(50ms);
  }

  StopRobot(publisher);

  if (!reached_goal)
  {
    std::cerr << "警告：15 秒内未达到目标，已安全停止。" << std::endl;
    return 2;
  }

  double final_x = 0.0;
  double final_y = 0.0;
  odom.GetPosition(final_x, final_y);

  const double final_distance = std::hypot(
      final_x - start_x,
      final_y - start_y);

  std::cout << "完成。" << std::endl;
  std::cout << "终点: x=" << final_x
            << ", y=" << final_y << std::endl;
  std::cout << "实际位移: "
            << final_distance << " 米" << std::endl;

  return 0;
}
