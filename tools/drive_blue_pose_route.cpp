#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool valid = false;
};

struct Waypoint {
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double yaw_deg = 0.0;
};

static std::atomic<bool> running{true};
static std::mutex pose_mutex;
static Pose2D latest_pose;

double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

double deg2rad(double d) {
  return d * M_PI / 180.0;
}

double yaw_from_quaternion(double w, double x, double y, double z) {
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void signal_handler(int) {
  running = false;
}

void odom_callback(const ignition::msgs::Odometry &msg) {
  std::lock_guard<std::mutex> lock(pose_mutex);

  latest_pose.x = msg.pose().position().x();
  latest_pose.y = msg.pose().position().y();

  const auto &q = msg.pose().orientation();
  latest_pose.yaw = yaw_from_quaternion(q.w(), q.x(), q.y(), q.z());
  latest_pose.valid = true;
}

Pose2D get_pose() {
  std::lock_guard<std::mutex> lock(pose_mutex);
  return latest_pose;
}

void publish_stop(ignition::transport::Node::Publisher &pub) {
  ignition::msgs::Twist cmd;
  cmd.mutable_linear()->set_x(0.0);
  cmd.mutable_angular()->set_z(0.0);

  for (int i = 0; i < 8; ++i) {
    pub.Publish(cmd);
    std::this_thread::sleep_for(80ms);
  }
}

std::vector<std::string> split_csv_line(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;

  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }

  return out;
}

std::vector<Waypoint> read_waypoints(const std::string &csv_path) {
  std::ifstream fin(csv_path);

  if (!fin) {
    throw std::runtime_error("无法打开路线 CSV: " + csv_path);
  }

  std::vector<Waypoint> waypoints;
  std::string line;

  std::getline(fin, line);

  while (std::getline(fin, line)) {
    if (line.empty()) {
      continue;
    }

    auto parts = split_csv_line(line);

    if (parts.size() < 4) {
      continue;
    }

    Waypoint wp;
    wp.name = parts[0];
    wp.x = std::stod(parts[1]);
    wp.y = std::stod(parts[2]);
    wp.yaw_deg = std::stod(parts[3]);

    waypoints.push_back(wp);
  }

  return waypoints;
}

bool execute_waypoint(
    ignition::transport::Node::Publisher &pub,
    const Waypoint &wp,
    std::ofstream &summary,
    std::ofstream &traj,
    double xy_tolerance,
    double yaw_tolerance_rad,
    double waypoint_timeout_sec) {

  const auto start_time = std::chrono::steady_clock::now();

  Pose2D start_pose = get_pose();

  std::string status = "timeout";
  double final_error = 0.0;
  double final_yaw_error = 0.0;

  bool position_reached = false;
  bool final_yaw_reached = false;

  while (running) {
    Pose2D p = get_pose();

    if (!p.valid) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    double dx = wp.x - p.x;
    double dy = wp.y - p.y;
    double dist = std::hypot(dx, dy);

    double target_heading = std::atan2(dy, dx);
    double heading_error = normalize_angle(target_heading - p.yaw);

    double target_yaw = deg2rad(wp.yaw_deg);
    double yaw_error = normalize_angle(target_yaw - p.yaw);

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time).count();

    ignition::msgs::Twist cmd;

    if (elapsed > waypoint_timeout_sec) {
      status = "timeout";
      break;
    }

    if (!position_reached) {
      if (dist <= xy_tolerance) {
        position_reached = true;
        publish_stop(pub);
        continue;
      }

      if (std::fabs(heading_error) > deg2rad(8.0)) {
        double w = std::clamp(1.2 * heading_error, -0.35, 0.35);
        cmd.mutable_linear()->set_x(0.0);
        cmd.mutable_angular()->set_z(w);
      } else {
        double v = std::clamp(0.45 * dist, 0.03, 0.18);
        double w = std::clamp(0.8 * heading_error, -0.25, 0.25);
        cmd.mutable_linear()->set_x(v);
        cmd.mutable_angular()->set_z(w);
      }
    } else {
      if (std::fabs(yaw_error) <= yaw_tolerance_rad) {
        final_yaw_reached = true;
        status = "success";
        break;
      }

      double w = std::clamp(1.0 * yaw_error, -0.30, 0.30);
      cmd.mutable_linear()->set_x(0.0);
      cmd.mutable_angular()->set_z(w);
    }

    pub.Publish(cmd);

    traj << wp.name << ","
         << std::fixed << std::setprecision(6)
         << p.x << "," << p.y << "," << rad2deg(p.yaw) << ","
         << wp.x << "," << wp.y << "," << wp.yaw_deg << ","
         << dist << "," << rad2deg(yaw_error) << ","
         << elapsed << "\n";

    std::cout << "航点: " << wp.name
              << " | 当前: (" << std::fixed << std::setprecision(3)
              << p.x << ", " << p.y << ")"
              << " | 剩余距离: " << dist
              << " m | 当前yaw: " << rad2deg(p.yaw)
              << " deg | 目标yaw: " << wp.yaw_deg
              << " deg | yaw误差: " << rad2deg(yaw_error)
              << " deg" << std::endl;

    std::this_thread::sleep_for(120ms);
  }

  publish_stop(pub);

  Pose2D end_pose = get_pose();
  final_error = std::hypot(wp.x - end_pose.x, wp.y - end_pose.y);
  final_yaw_error = normalize_angle(deg2rad(wp.yaw_deg) - end_pose.yaw);

  double duration = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();

  summary << wp.name << ","
          << wp.x << "," << wp.y << "," << wp.yaw_deg << ","
          << start_pose.x << "," << start_pose.y << "," << rad2deg(start_pose.yaw) << ","
          << end_pose.x << "," << end_pose.y << "," << rad2deg(end_pose.yaw) << ","
          << final_error << "," << rad2deg(final_yaw_error) << ","
          << duration << "," << status << "\n";

  std::cout << "航点完成: " << wp.name
            << " | 位置误差=" << final_error
            << " m | yaw误差=" << rad2deg(final_yaw_error)
            << " deg | 状态=" << status << std::endl;

  return status == "success";
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "用法: " << argv[0]
              << " <route_csv> <output_dir>" << std::endl;
    return 1;
  }

  std::signal(SIGINT, signal_handler);

  const std::string route_csv = argv[1];
  const std::string output_dir = argv[2];

  std::string mkdir_cmd = "mkdir -p " + output_dir;
  std::system(mkdir_cmd.c_str());

  auto waypoints = read_waypoints(route_csv);

  if (waypoints.empty()) {
    std::cerr << "路线文件中没有有效航点。" << std::endl;
    return 1;
  }

  ignition::transport::Node node;

  auto cmd_pub = node.Advertise<ignition::msgs::Twist>(
      "/model/vehicle_blue/cmd_vel");

  node.Subscribe("/model/vehicle_blue/odometry", odom_callback);

  std::cout << "等待 odometry..." << std::endl;

  for (int i = 0; i < 50; ++i) {
    if (get_pose().valid) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }

  if (!get_pose().valid) {
    std::cerr << "没有收到 odometry，退出。" << std::endl;
    return 1;
  }

  std::ofstream summary(output_dir + "/pose_route_summary.csv");
  std::ofstream traj(output_dir + "/pose_route_trajectory.csv");

  summary << "waypoint_name,target_x,target_y,target_yaw_deg,"
          << "start_x,start_y,start_yaw_deg,end_x,end_y,end_yaw_deg,"
          << "final_error_m,final_yaw_error_deg,duration_sec,status\n";

  traj << "waypoint_name,x,y,yaw_deg,target_x,target_y,target_yaw_deg,"
       << "remaining_distance_m,yaw_error_deg,elapsed_sec\n";

  const double xy_tolerance = 0.025;
  const double yaw_tolerance_rad = deg2rad(3.0);
  const double timeout_sec = 90.0;

  int success_count = 0;

  for (const auto &wp : waypoints) {
    bool ok = execute_waypoint(
        cmd_pub,
        wp,
        summary,
        traj,
        xy_tolerance,
        yaw_tolerance_rad,
        timeout_sec);

    if (ok) {
      ++success_count;
    } else {
      std::cout << "航点失败，停止后续路线。" << std::endl;
      break;
    }
  }

  publish_stop(cmd_pub);

  std::cout << "EXP-015 pose route 执行完成: "
            << success_count << " / " << waypoints.size()
            << " success" << std::endl;

  std::cout << "summary: " << output_dir << "/pose_route_summary.csv" << std::endl;
  std::cout << "trajectory: " << output_dir << "/pose_route_trajectory.csv" << std::endl;

  return 0;
}
