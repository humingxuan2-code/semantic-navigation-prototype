#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
const std::string kCmdTopic = "/model/vehicle_blue/cmd_vel";
const std::string kOdomTopic = "/model/vehicle_blue/odometry";

constexpr double kGoalTolerance = 0.025;
constexpr double kHeadingTolerance = 0.10;

constexpr double kMaxLinearSpeed = 0.18;
constexpr double kMinLinearSpeed = 0.025;
constexpr double kMaxAngularSpeed = 0.80;
constexpr double kMinAngularSpeed = 0.08;

constexpr double kDistanceGain = 0.55;
constexpr double kTurnGain = 1.50;
constexpr double kForwardYawGain = 1.20;
constexpr double kForwardMaxAngularSpeed = 0.35;

/*
 * EXP-013 uses longer obstacle-detour segments.
 * Keep a safety timeout, but allow enough time for long global waypoints.
 */
constexpr double kWaypointTimeoutSec = 70.0;

struct Waypoint
{
  std::string name;
  double x;
  double y;
};

struct WaypointResult
{
  std::string name;
  double target_x;
  double target_y;
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double final_error;
  double duration_sec;
  bool success;
};

double NormalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }

  return angle;
}

double RadToDeg(double radians)
{
  return radians * 180.0 / M_PI;
}

std::string Trim(const std::string &text)
{
  const std::string whitespace = " \t\r\n";

  const std::size_t first = text.find_first_not_of(whitespace);

  if (first == std::string::npos)
  {
    return "";
  }

  const std::size_t last = text.find_last_not_of(whitespace);

  return text.substr(first, last - first + 1);
}

void SetTwist(
  ignition::msgs::Twist &message,
  double linear_x,
  double angular_z)
{
  message.mutable_linear()->set_x(linear_x);
  message.mutable_linear()->set_y(0.0);
  message.mutable_linear()->set_z(0.0);

  message.mutable_angular()->set_x(0.0);
  message.mutable_angular()->set_y(0.0);
  message.mutable_angular()->set_z(angular_z);
}

class OdomListener
{
public:
  void OnOdom(const ignition::msgs::Odometry &message)
  {
    const auto &position = message.pose().position();
    const auto &orientation = message.pose().orientation();

    const double siny_cosp =
      2.0 * (
        orientation.w() * orientation.z() +
        orientation.x() * orientation.y()
      );

    const double cosy_cosp =
      1.0 - 2.0 * (
        orientation.y() * orientation.y() +
        orientation.z() * orientation.z()
      );

    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    {
      std::lock_guard<std::mutex> lock(this->mutex_);

      this->x_ = position.x();
      this->y_ = position.y();
      this->yaw_ = yaw;
      this->received_ = true;
    }

    this->cv_.notify_all();
  }

  bool WaitForFirstMessage(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(this->mutex_);

    return this->cv_.wait_for(
      lock,
      timeout,
      [this]()
      {
        return this->received_;
      }
    );
  }

  bool GetPose(double &x, double &y, double &yaw)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->received_)
    {
      return false;
    }

    x = this->x_;
    y = this->y_;
    yaw = this->yaw_;

    return true;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;

  bool received_ = false;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

std::vector<Waypoint> LoadRoute(const std::filesystem::path &route_path)
{
  std::ifstream input(route_path);

  if (!input.is_open())
  {
    throw std::runtime_error(
      "无法打开路线文件：" + route_path.string()
    );
  }

  std::string line;

  if (!std::getline(input, line))
  {
    throw std::runtime_error("路线文件为空。");
  }

  std::vector<Waypoint> waypoints;

  while (std::getline(input, line))
  {
    if (Trim(line).empty())
    {
      continue;
    }

    std::stringstream stream(line);

    std::string name;
    std::string x_text;
    std::string y_text;

    if (
      !std::getline(stream, name, ',') ||
      !std::getline(stream, x_text, ',') ||
      !std::getline(stream, y_text, ',')
    )
    {
      throw std::runtime_error(
        "路线文件格式错误，期望：waypoint_name,target_x,target_y"
      );
    }

    waypoints.push_back(
      {
        Trim(name),
        std::stod(Trim(x_text)),
        std::stod(Trim(y_text))
      }
    );
  }

  if (waypoints.empty())
  {
    throw std::runtime_error("路线文件中没有有效航点。");
  }

  return waypoints;
}

template <typename Publisher>
void PublishStop(Publisher &publisher)
{
  ignition::msgs::Twist stop_message;
  SetTwist(stop_message, 0.0, 0.0);

  for (int index = 0; index < 12; ++index)
  {
    publisher.Publish(stop_message);
    std::this_thread::sleep_for(30ms);
  }
}

void WriteTrajectorySample(
  std::ofstream &output,
  double elapsed_sec,
  int waypoint_index,
  const std::string &waypoint_name,
  double x,
  double y,
  double yaw,
  double target_x,
  double target_y,
  double distance,
  double heading_error,
  double linear_speed,
  double angular_speed,
  const std::string &mode)
{
  output
    << std::fixed
    << std::setprecision(6)
    << elapsed_sec << ","
    << waypoint_index << ","
    << waypoint_name << ","
    << x << ","
    << y << ","
    << RadToDeg(yaw) << ","
    << target_x << ","
    << target_y << ","
    << distance << ","
    << RadToDeg(heading_error) << ","
    << linear_speed << ","
    << angular_speed << ","
    << mode
    << "\n";
}

template <typename Publisher>
WaypointResult NavigateToWaypoint(
  int waypoint_index,
  const Waypoint &waypoint,
  OdomListener &odom,
  Publisher &publisher,
  std::ofstream &trajectory_output,
  const std::chrono::steady_clock::time_point &route_start)
{
  double start_x = 0.0;
  double start_y = 0.0;
  double start_yaw = 0.0;

  if (!odom.GetPose(start_x, start_y, start_yaw))
  {
    throw std::runtime_error("无法读取蓝车当前 odometry。");
  }

  std::cout << "\n============================================\n";
  std::cout << "航点 " << waypoint_index + 1
            << "：" << waypoint.name << "\n";
  std::cout << "当前起点：x=" << std::fixed << std::setprecision(3)
            << start_x << ", y=" << start_y
            << ", yaw=" << RadToDeg(start_yaw) << " 度\n";
  std::cout << "全局目标：x=" << waypoint.x
            << ", y=" << waypoint.y << "\n";

  const auto waypoint_start = std::chrono::steady_clock::now();
  auto next_report = waypoint_start;

  while (true)
  {
    const auto now = std::chrono::steady_clock::now();

    const double waypoint_duration =
      std::chrono::duration<double>(now - waypoint_start).count();

    const double route_elapsed =
      std::chrono::duration<double>(now - route_start).count();

    if (waypoint_duration > kWaypointTimeoutSec)
    {
      PublishStop(publisher);

      double end_x = 0.0;
      double end_y = 0.0;
      double end_yaw = 0.0;

      odom.GetPose(end_x, end_y, end_yaw);

      const double final_error = std::hypot(
        waypoint.x - end_x,
        waypoint.y - end_y
      );

      std::cout << "警告：航点超时，已安全停车。\n";

      return {
        waypoint.name,
        waypoint.x,
        waypoint.y,
        start_x,
        start_y,
        end_x,
        end_y,
        final_error,
        waypoint_duration,
        false
      };
    }

    double current_x = 0.0;
    double current_y = 0.0;
    double current_yaw = 0.0;

    if (!odom.GetPose(current_x, current_y, current_yaw))
    {
      std::this_thread::sleep_for(50ms);
      continue;
    }

    const double dx = waypoint.x - current_x;
    const double dy = waypoint.y - current_y;

    const double distance = std::hypot(dx, dy);
    const double target_yaw = std::atan2(dy, dx);

    const double heading_error =
      NormalizeAngle(target_yaw - current_yaw);

    double linear_speed = 0.0;
    double angular_speed = 0.0;
    std::string mode = "turn";

    if (distance <= kGoalTolerance)
    {
      WriteTrajectorySample(
        trajectory_output,
        route_elapsed,
        waypoint_index,
        waypoint.name,
        current_x,
        current_y,
        current_yaw,
        waypoint.x,
        waypoint.y,
        distance,
        heading_error,
        0.0,
        0.0,
        "reached"
      );

      PublishStop(publisher);

      double end_x = 0.0;
      double end_y = 0.0;
      double end_yaw = 0.0;

      odom.GetPose(end_x, end_y, end_yaw);

      const double final_error = std::hypot(
        waypoint.x - end_x,
        waypoint.y - end_y
      );

      std::cout << "航点到达，最终误差："
                << std::setprecision(4)
                << final_error
                << " m\n";

      return {
        waypoint.name,
        waypoint.x,
        waypoint.y,
        start_x,
        start_y,
        end_x,
        end_y,
        final_error,
        waypoint_duration,
        true
      };
    }

    if (std::abs(heading_error) > kHeadingTolerance)
    {
      angular_speed = std::clamp(
        kTurnGain * heading_error,
        -kMaxAngularSpeed,
        kMaxAngularSpeed
      );

      if (std::abs(angular_speed) < kMinAngularSpeed)
      {
        angular_speed = std::copysign(
          kMinAngularSpeed,
          heading_error
        );
      }

      mode = "turn";
    }
    else
    {
      linear_speed = std::clamp(
        kDistanceGain * distance,
        kMinLinearSpeed,
        kMaxLinearSpeed
      );

      angular_speed = std::clamp(
        kForwardYawGain * heading_error,
        -kForwardMaxAngularSpeed,
        kForwardMaxAngularSpeed
      );

      mode = "forward";
    }

    ignition::msgs::Twist command;
    SetTwist(command, linear_speed, angular_speed);
    publisher.Publish(command);

    WriteTrajectorySample(
      trajectory_output,
      route_elapsed,
      waypoint_index,
      waypoint.name,
      current_x,
      current_y,
      current_yaw,
      waypoint.x,
      waypoint.y,
      distance,
      heading_error,
      linear_speed,
      angular_speed,
      mode
    );

    if (now >= next_report)
    {
      std::cout
        << "模式：" << mode
        << " | 当前：("
        << std::setprecision(3)
        << current_x << ", "
        << current_y << ")"
        << " | 剩余距离："
        << distance << " m"
        << " | 朝向误差："
        << std::setprecision(2)
        << RadToDeg(heading_error)
        << " 度"
        << " | v=" << std::setprecision(3)
        << linear_speed
        << " m/s"
        << " | w=" << angular_speed
        << " rad/s\n";

      next_report += 300ms;
    }

    std::this_thread::sleep_for(50ms);
  }
}
}

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cerr
      << "用法："
      << argv[0]
      << " <route.csv> <output_directory>\n";

    return 1;
  }

  try
  {
    const std::filesystem::path route_path = argv[1];
    const std::filesystem::path output_dir = argv[2];

    const std::vector<Waypoint> waypoints =
      LoadRoute(route_path);

    std::filesystem::create_directories(output_dir);

    const std::filesystem::path trajectory_path =
      output_dir / "global_route_trajectory.csv";

    const std::filesystem::path summary_path =
      output_dir / "global_route_summary.csv";

    std::ofstream trajectory_output(trajectory_path);

    if (!trajectory_output.is_open())
    {
      throw std::runtime_error(
        "无法创建轨迹文件：" + trajectory_path.string()
      );
    }

    trajectory_output
      << "time_sec,waypoint_index,waypoint_name,"
      << "x,y,yaw_deg,target_x,target_y,"
      << "distance_to_goal,heading_error_deg,"
      << "linear_speed,angular_speed,mode\n";

    ignition::transport::Node node;
    OdomListener odom;

    node.Subscribe(
      kOdomTopic,
      &OdomListener::OnOdom,
      &odom
    );

    auto publisher =
      node.Advertise<ignition::msgs::Twist>(kCmdTopic);

    std::cout << "等待蓝车 odometry...\n";

    if (!odom.WaitForFirstMessage(5000ms))
    {
      std::cerr
        << "未在 5 秒内收到 odometry。"
        << "请确认 Gazebo 正在运行。\n";

      return 2;
    }

    std::cout << "已读取路线文件，共 "
              << waypoints.size()
              << " 个全局航点。\n";

    const auto route_start = std::chrono::steady_clock::now();

    std::vector<WaypointResult> results;
    bool all_success = true;

    for (
      std::size_t index = 0;
      index < waypoints.size();
      ++index
    )
    {
      WaypointResult result = NavigateToWaypoint(
        static_cast<int>(index),
        waypoints[index],
        odom,
        publisher,
        trajectory_output,
        route_start
      );

      results.push_back(result);

      if (!result.success)
      {
        all_success = false;
        break;
      }

      std::this_thread::sleep_for(400ms);
    }

    PublishStop(publisher);

    std::ofstream summary_output(summary_path);

    if (!summary_output.is_open())
    {
      throw std::runtime_error(
        "无法创建汇总文件：" + summary_path.string()
      );
    }

    summary_output
      << "waypoint_name,target_x,target_y,"
      << "start_x,start_y,end_x,end_y,"
      << "final_error_m,duration_sec,status\n";

    std::cout << "\n============================================\n";
    std::cout << "全局路线执行结果\n";

    for (const auto &result : results)
    {
      summary_output
        << result.name << ","
        << std::fixed
        << std::setprecision(6)
        << result.target_x << ","
        << result.target_y << ","
        << result.start_x << ","
        << result.start_y << ","
        << result.end_x << ","
        << result.end_y << ","
        << result.final_error << ","
        << result.duration_sec << ","
        << (result.success ? "success" : "timeout")
        << "\n";

      std::cout
        << result.name
        << " | 误差="
        << std::setprecision(4)
        << result.final_error
        << " m"
        << " | 耗时="
        << std::setprecision(2)
        << result.duration_sec
        << " s"
        << " | "
        << (result.success ? "成功" : "超时")
        << "\n";
    }

    std::cout << "\n轨迹文件："
              << trajectory_path
              << "\n汇总文件："
              << summary_path
              << "\n";

    return all_success ? 0 : 3;
  }
  catch (const std::exception &error)
  {
    std::cerr << "错误：" << error.what() << "\n";
    return 1;
  }
}
