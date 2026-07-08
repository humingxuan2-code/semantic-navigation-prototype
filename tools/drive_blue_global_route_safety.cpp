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
};

static std::atomic<bool> running{true};
static std::mutex pose_mutex;
static Pose2D latest_pose;

constexpr double ROBOT_LENGTH = 2.20;
constexpr double ROBOT_WIDTH = 1.80;

constexpr double OBSTACLE_CENTER_X = 3.5;
constexpr double OBSTACLE_CENTER_Y = 0.0;
constexpr double OBSTACLE_SIZE_X = 1.4;
constexpr double OBSTACLE_SIZE_Y = 1.6;

constexpr double SAFETY_STOP_THRESHOLD = 0.25;
constexpr double XY_TOLERANCE = 0.025;
constexpr double TIMEOUT_SEC = 120.0;

double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double yaw_from_quaternion(double w, double x, double y, double z) {
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double rad2deg(double r) {
  return r * 180.0 / M_PI;
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

  for (int i = 0; i < 10; ++i) {
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
    if (line.empty()) continue;

    auto parts = split_csv_line(line);

    if (parts.size() < 3) continue;

    Waypoint wp;
    wp.name = parts[0];
    wp.x = std::stod(parts[1]);
    wp.y = std::stod(parts[2]);
    waypoints.push_back(wp);
  }

  return waypoints;
}

struct Point {
  double x;
  double y;
};

std::vector<Point> robot_corners(double x, double y, double yaw) {
  double hl = ROBOT_LENGTH / 2.0;
  double hw = ROBOT_WIDTH / 2.0;

  std::vector<Point> local = {
      {hl, hw},
      {hl, -hw},
      {-hl, -hw},
      {-hl, hw},
  };

  double c = std::cos(yaw);
  double s = std::sin(yaw);

  std::vector<Point> world;

  for (const auto &p : local) {
    world.push_back({
        x + p.x * c - p.y * s,
        y + p.x * s + p.y * c,
    });
  }

  return world;
}

std::vector<Point> obstacle_corners() {
  double x0 = OBSTACLE_CENTER_X - OBSTACLE_SIZE_X / 2.0;
  double x1 = OBSTACLE_CENTER_X + OBSTACLE_SIZE_X / 2.0;
  double y0 = OBSTACLE_CENTER_Y - OBSTACLE_SIZE_Y / 2.0;
  double y1 = OBSTACLE_CENTER_Y + OBSTACLE_SIZE_Y / 2.0;

  return {
      {x0, y0},
      {x1, y0},
      {x1, y1},
      {x0, y1},
  };
}

double dot(Point a, Point b) {
  return a.x * b.x + a.y * b.y;
}

std::vector<Point> polygon_axes(const std::vector<Point> &poly) {
  std::vector<Point> axes;

  for (size_t i = 0; i < poly.size(); ++i) {
    Point p1 = poly[i];
    Point p2 = poly[(i + 1) % poly.size()];

    Point edge{p2.x - p1.x, p2.y - p1.y};
    Point normal{-edge.y, edge.x};

    double len = std::hypot(normal.x, normal.y);

    if (len > 1e-9) {
      axes.push_back({normal.x / len, normal.y / len});
    }
  }

  return axes;
}

void project_polygon(const std::vector<Point> &poly, Point axis, double &mn, double &mx) {
  mn = dot(poly[0], axis);
  mx = mn;

  for (const auto &p : poly) {
    double v = dot(p, axis);
    mn = std::min(mn, v);
    mx = std::max(mx, v);
  }
}

bool polygons_intersect(const std::vector<Point> &a, const std::vector<Point> &b) {
  auto axes_a = polygon_axes(a);
  auto axes_b = polygon_axes(b);

  axes_a.insert(axes_a.end(), axes_b.begin(), axes_b.end());

  for (const auto &axis : axes_a) {
    double amin, amax, bmin, bmax;
    project_polygon(a, axis, amin, amax);
    project_polygon(b, axis, bmin, bmax);

    if (amax < bmin || bmax < amin) {
      return false;
    }
  }

  return true;
}

double point_segment_distance(Point p, Point a, Point b) {
  double abx = b.x - a.x;
  double aby = b.y - a.y;
  double apx = p.x - a.x;
  double apy = p.y - a.y;

  double denom = abx * abx + aby * aby;

  if (denom <= 1e-12) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }

  double t = std::clamp((apx * abx + apy * aby) / denom, 0.0, 1.0);
  double cx = a.x + t * abx;
  double cy = a.y + t * aby;

  return std::hypot(p.x - cx, p.y - cy);
}

double polygon_distance(const std::vector<Point> &a, const std::vector<Point> &b) {
  if (polygons_intersect(a, b)) {
    return 0.0;
  }

  double best = 1e9;

  for (const auto &p : a) {
    for (size_t i = 0; i < b.size(); ++i) {
      best = std::min(best, point_segment_distance(p, b[i], b[(i + 1) % b.size()]));
    }
  }

  for (const auto &p : b) {
    for (size_t i = 0; i < a.size(); ++i) {
      best = std::min(best, point_segment_distance(p, a[i], a[(i + 1) % a.size()]));
    }
  }

  return best;
}

double footprint_clearance(double x, double y, double yaw) {
  auto robot = robot_corners(x, y, yaw);
  auto obstacle = obstacle_corners();
  return polygon_distance(robot, obstacle);
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "用法: " << argv[0] << " <route_csv> <output_dir>" << std::endl;
    return 1;
  }

  std::signal(SIGINT, signal_handler);

  std::string route_csv = argv[1];
  std::string output_dir = argv[2];

  std::system(("mkdir -p " + output_dir).c_str());

  auto waypoints = read_waypoints(route_csv);

  if (waypoints.empty()) {
    std::cerr << "路线文件没有有效航点。" << std::endl;
    return 1;
  }

  ignition::transport::Node node;
  auto cmd_pub = node.Advertise<ignition::msgs::Twist>("/model/vehicle_blue/cmd_vel");
  node.Subscribe("/model/vehicle_blue/odometry", odom_callback);

  std::cout << "等待 odometry..." << std::endl;

  for (int i = 0; i < 50; ++i) {
    if (get_pose().valid) break;
    std::this_thread::sleep_for(100ms);
  }

  if (!get_pose().valid) {
    std::cerr << "没有收到 odometry，退出。" << std::endl;
    return 1;
  }

  std::ofstream summary(output_dir + "/safety_route_summary.csv");
  std::ofstream traj(output_dir + "/safety_route_trajectory.csv");

  summary << "waypoint_name,target_x,target_y,start_x,start_y,end_x,end_y,"
          << "final_error_m,min_clearance_m,min_clearance_mm,duration_sec,status\n";

  traj << "waypoint_name,x,y,yaw_deg,target_x,target_y,remaining_distance_m,"
       << "footprint_clearance_m,footprint_clearance_mm,elapsed_sec\n";

  int success_count = 0;
  bool stopped_by_safety = false;

  for (const auto &wp : waypoints) {
    auto start_time = std::chrono::steady_clock::now();
    Pose2D start_pose = get_pose();

    std::string status = "timeout";
    double min_clearance = 1e9;

    while (running) {
      Pose2D p = get_pose();

      if (!p.valid) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      double clearance = footprint_clearance(p.x, p.y, p.yaw);
      min_clearance = std::min(min_clearance, clearance);

      double dx = wp.x - p.x;
      double dy = wp.y - p.y;
      double dist = std::hypot(dx, dy);

      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time).count();

      traj << wp.name << ","
           << std::fixed << std::setprecision(6)
           << p.x << "," << p.y << "," << rad2deg(p.yaw) << ","
           << wp.x << "," << wp.y << ","
           << dist << "," << clearance << "," << clearance * 1000.0 << ","
           << elapsed << "\n";

      if (clearance <= SAFETY_STOP_THRESHOLD) {
        status = "safety_stop";
        stopped_by_safety = true;
        publish_stop(cmd_pub);

        std::cout << "安全停车触发: clearance="
                  << std::fixed << std::setprecision(3)
                  << clearance << " m, threshold="
                  << SAFETY_STOP_THRESHOLD << " m" << std::endl;
        break;
      }

      if (dist <= XY_TOLERANCE) {
        status = "success";
        publish_stop(cmd_pub);
        break;
      }

      if (elapsed > TIMEOUT_SEC) {
        status = "timeout";
        publish_stop(cmd_pub);
        break;
      }

      double target_heading = std::atan2(dy, dx);
      double heading_error = normalize_angle(target_heading - p.yaw);

      ignition::msgs::Twist cmd;

      if (std::fabs(heading_error) > M_PI / 18.0) {
        double w = std::clamp(1.2 * heading_error, -0.35, 0.35);
        cmd.mutable_linear()->set_x(0.0);
        cmd.mutable_angular()->set_z(w);
      } else {
        double v = std::clamp(0.45 * dist, 0.03, 0.18);
        double w = std::clamp(0.8 * heading_error, -0.25, 0.25);
        cmd.mutable_linear()->set_x(v);
        cmd.mutable_angular()->set_z(w);
      }

      cmd_pub.Publish(cmd);

      std::cout << "航点: " << wp.name
                << " | 当前: (" << std::fixed << std::setprecision(3)
                << p.x << ", " << p.y << ")"
                << " | 剩余距离: " << dist
                << " m | clearance: " << clearance
                << " m" << std::endl;

      std::this_thread::sleep_for(120ms);
    }

    Pose2D end_pose = get_pose();
    double final_error = std::hypot(wp.x - end_pose.x, wp.y - end_pose.y);
    double duration = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

    summary << wp.name << ","
            << wp.x << "," << wp.y << ","
            << start_pose.x << "," << start_pose.y << ","
            << end_pose.x << "," << end_pose.y << ","
            << final_error << ","
            << min_clearance << "," << min_clearance * 1000.0 << ","
            << duration << "," << status << "\n";

    if (status == "success") {
      success_count++;
    } else {
      break;
    }
  }

  publish_stop(cmd_pub);

  std::cout << "EXP-017 safety-guard route 执行结束: "
            << success_count << " / " << waypoints.size()
            << " success";

  if (stopped_by_safety) {
    std::cout << "，已触发安全停车";
  }

  std::cout << std::endl;

  std::cout << "summary: " << output_dir << "/safety_route_summary.csv" << std::endl;
  std::cout << "trajectory: " << output_dir << "/safety_route_trajectory.csv" << std::endl;

  return 0;
}
