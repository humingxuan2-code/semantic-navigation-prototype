#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

using namespace std::chrono_literals;

namespace
{
const std::string kTopic = "/model/vehicle_blue/cmd_vel";

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
void ShortStop(Publisher &publisher)
{
  PublishFor(publisher, 0.0, 0.0, 500ms);
}

template <typename Publisher>
void StopRobot(Publisher &publisher)
{
  // 最终连续发送 2 秒零速度，确保车稳定停住
  PublishFor(publisher, 0.0, 0.0, 2000ms);
}
}  // namespace

int main(int argc, char **argv)
{
  const std::string action = (argc >= 2) ? argv[1] : "forward";

  ignition::transport::Node node;
  auto publisher = node.Advertise<ignition::msgs::Twist>(kTopic);

  if (!publisher)
  {
    std::cerr << "无法创建 Gazebo 发布器: " << kTopic << std::endl;
    return 1;
  }

  // 等待 Gazebo 的 DiffDrive 控制插件发现发布器
  std::this_thread::sleep_for(1000ms);

  if (action == "forward")
  {
    std::cout << "蓝车前进 3 秒..." << std::endl;
    PublishFor(publisher, 0.30, 0.0, 3000ms);
  }
  else if (action == "left")
  {
    std::cout << "蓝车原地左转约 90 度..." << std::endl;
    PublishFor(publisher, 0.0, 1.0, 1550ms);
  }
  else if (action == "right")
  {
    std::cout << "蓝车原地右转约 90 度..." << std::endl;
    PublishFor(publisher, 0.0, -1.0, 1550ms);
  }
  else if (action == "route")
  {
    std::cout << "开始执行 L 形路线..." << std::endl;

    std::cout << "步骤 1：前进 2 秒" << std::endl;
    PublishFor(publisher, 0.25, 0.0, 2000ms);
    ShortStop(publisher);

    std::cout << "步骤 2：原地左转约 90 度" << std::endl;
    PublishFor(publisher, 0.0, 1.0, 1550ms);
    ShortStop(publisher);

    std::cout << "步骤 3：再次前进 2 秒" << std::endl;
    PublishFor(publisher, 0.25, 0.0, 2000ms);
  }
  else if (action == "stop")
  {
    std::cout << "蓝车停止..." << std::endl;
  }
  else
  {
    std::cerr << "用法: " << argv[0]
              << " {forward|left|right|route|stop}" << std::endl;
    return 1;
  }

  StopRobot(publisher);
  std::cout << "完成：已连续发送停止指令。" << std::endl;
  return 0;
}
