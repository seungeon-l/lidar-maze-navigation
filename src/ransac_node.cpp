#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar_maze_navigation/msg/ransac.hpp"
#include "lidar_maze_navigation/msg/line.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <chrono>

// Robot geometry
const float r = 0.032;
const float b = 0.260;
const float delta = 0.14;

// Grid geometry
const float CELL_SIZE = 0.80;
const float HALF_CELL = 0.40;

// RANSAC types & constants
struct Point {
  float x, y;
  int   beam_idx;
};

struct Line {
  float alpha;
  float d;
  int   n_inliers;
};

const float INLIER_THRESHOLD = 0.008f;
const int   MIN_INLIERS      = 70;
const int   RANSAC_ITERS     = 700;
const int   MAX_WALLS        = 4;
const int   MAX_BEAM_GAP     = 30;

// Globals
sensor_msgs::msg::LaserScan latest_scan;
bool scan_received = false;

void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
  latest_scan    = msg;
  scan_received  = true;
}


// RANSAC helpers
std::vector<Point> scan_to_points(const sensor_msgs::msg::LaserScan &scan) {
  std::vector<Point> pts;
  pts.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); i++) {
    float rng = scan.ranges[i];
    if (!std::isfinite(rng)) continue;
    if (rng < scan.range_min || rng > scan.range_max) continue;
    float ang = scan.angle_min + i * scan.angle_increment;
    pts.push_back({rng * std::cos(ang), rng * std::sin(ang), (int)i});
  }
  return pts;
}

float point_line_distance(float px, float py,
                          float ax, float ay,
                          float bx, float by) {
  float dx  = bx - ax;
  float dy  = by - ay;

  float len = std::sqrt(dx * dx + dy * dy);
  if (len < 1e-6f) return std::numeric_limits<float>::infinity();
  return std::fabs(dx * (ay - py) - dy * (ax - px)) / len;
}

bool inliers_contiguous(const std::vector<int> &idxs) {
  if (idxs.size() < 2) return true;
  std::vector<int> sorted = idxs;
  std::sort(sorted.begin(), sorted.end());
  for (size_t i = 1; i < sorted.size(); i++)
    if (sorted[i] - sorted[i - 1] > MAX_BEAM_GAP) return false;
  return true;
}

Line ransac_line(rclcpp::Node::SharedPtr node,
                 const std::vector<Point> &pts,
                 std::mt19937 &rng) {
  Line best{0.0f, 0.0f, 0};
  if (pts.size() < (size_t)MIN_INLIERS) return best;

  std::uniform_int_distribution<int> pick(0, (int)pts.size() - 1);
  std::vector<int> inlier_idxs;
  inlier_idxs.reserve(pts.size());

  for (int it = 0; it < RANSAC_ITERS; it++) {
    int i = pick(rng), j = pick(rng);
    if (i == j) continue;
    const Point &a = pts[i];
    const Point &b = pts[j];

    inlier_idxs.clear();
    for (size_t k = 0; k < pts.size(); k++) {
      if (point_line_distance(pts[k].x, pts[k].y, a.x, a.y, b.x, b.y)
          <= INLIER_THRESHOLD)
        inlier_idxs.push_back((int)k);
    }
    if ((int)inlier_idxs.size() < MIN_INLIERS)       continue;
    if ((int)inlier_idxs.size() <= best.n_inliers)   continue;

    std::vector<int> beams;
    for (int idx : inlier_idxs) beams.push_back(pts[idx].beam_idx);
    if (!inliers_contiguous(beams)) continue;

    float dx    = b.x - a.x;
    float dy    = b.y - a.y;
    float alpha = std::atan2(dx, -dy);
    float d     = a.x * std::cos(alpha) + a.y * std::sin(alpha);
    if (d < 0) { d = -d; alpha += M_PI; }
    while (alpha <  0)         alpha += 2 * M_PI;
    while (alpha >= 2 * M_PI)  alpha -= 2 * M_PI;

    best = Line{alpha, d, (int)inlier_idxs.size()};
  }
  return best;
}

std::vector<Line> extract_walls(rclcpp::Node::SharedPtr node,
                                std::vector<Point> pts,
                                std::mt19937 &rng) {
  std::vector<Line> walls;
  for (int w = 0; w < MAX_WALLS; w++) {
    Line line = ransac_line(node, pts, rng);
    if (line.n_inliers < MIN_INLIERS) break;
    walls.push_back(line);
    std::vector<Point> remaining;
    remaining.reserve(pts.size());
    for (const Point &p : pts) {
      float dist = std::fabs(p.x * std::cos(line.alpha)
                           + p.y * std::sin(line.alpha) - line.d);
      if (dist > INLIER_THRESHOLD) remaining.push_back(p);
    }
    pts = std::move(remaining);
  }
  return walls;
}

std::vector<Line> do_scan_and_detect_walls(rclcpp::Node::SharedPtr node,
                                           std::mt19937 &rng) {
  auto pts = scan_to_points(latest_scan);

  // --- RANSAC ---
  auto walls = extract_walls(node, pts, rng);

  // --- delta correction & sort ---
  for (auto &w : walls)
    w.d += std::fabs(delta * std::cos(w.alpha));

  std::sort(walls.begin(), walls.end(),
            [](const Line &a, const Line &b) { return a.d < b.d; });

  return walls;
}


// main
int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ransac_node");

    auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", 10,
    [&](const sensor_msgs::msg::LaserScan &msg) { laser_callback(msg); });

    auto ransac_pub = node->create_publisher<lidar_maze_navigation::msg::Ransac>(
    "ransac", 10);

    rclcpp::Rate rate(100);
    std::random_device rd;
    std::mt19937 rng(rd());

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (scan_received) {
            scan_received = false;
            auto walls = do_scan_and_detect_walls(node, rng);

            // Publish
            lidar_maze_navigation::msg::Ransac msg;
            msg.walls.reserve(walls.size());
            for (const auto &w : walls) {
                lidar_maze_navigation::msg::Line line_msg;
                line_msg.angle    = w.alpha;
                line_msg.distance = w.d;
                msg.walls.push_back(line_msg);
            }
            ransac_pub->publish(msg);

            for (size_t i = 0; i < walls.size(); i++) {
                RCLCPP_INFO(node->get_logger(),
                            "wall %zu: angle=%.3f distance=%.3f inliers=%d",
                            i, walls[i].alpha, walls[i].d, walls[i].n_inliers);
            }
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
