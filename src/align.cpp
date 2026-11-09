#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <string>

// Robot geometry
const float r = 0.032;
const float b = 0.260;
const float delta = 0.14;

// Grid geometry
const float CELL_SIZE = 0.80;
const float HALF_CELL = 0.40;

// globals
float encoder_left  = 0.0;
float encoder_right = 0.0;
float current_heading = 0.0;

sensor_msgs::msg::LaserScan latest_scan;
bool scan_received = false;

// callbacks 
void sensor_callback(const create_fundamentals::msg::SensorPacket &msg) {
  encoder_left  = msg.encoder_left;
  encoder_right = msg.encoder_right;
}

void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
  latest_scan    = msg;
  scan_received  = true;
}

// sound helpers
void initialize_sounds() {
  std::string store_success = 
      "ros2 service call /store_song create_fundamentals/srv/StoreSong "
      "\"{number: 2, song: [72, 16, 76, 16, 79, 16, 84, 32]}\"";
      
  std::string store_failure = 
      "ros2 service call /store_song create_fundamentals/srv/StoreSong "
      "\"{number: 3, song: [60, 32, 55, 48]}\"";

  std::cout << "Attempting to store success sound..." << std::endl;
  std::system(store_success.c_str());
  
  std::cout << "Attempting to store failure sound..." << std::endl;
  std::system(store_failure.c_str());
}

void play_success_sound() {
  std::system("ros2 service call /play_song create_fundamentals/srv/PlaySong \"{number: 2}\"");
}

void play_failure_sound() {
  std::system("ros2 service call /play_song create_fundamentals/srv/PlaySong \"{number: 3}\"");
}


// drive helpers 
void send_drive(rclcpp::Node::SharedPtr node, float left, float right) {
  auto request  = std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
  auto diffDrive = node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive");
  while (!diffDrive->wait_for_service(std::chrono::seconds(1)))
    RCLCPP_INFO(node->get_logger(), "waiting for diff_drive service...");
  request->left  = left;
  request->right = right;
  diffDrive->async_send_request(request);
}

void turn_180(rclcpp::Node::SharedPtr node) {
  send_drive(node, 3.2, -3.2);
  rclcpp::sleep_for(std::chrono::seconds(4));
  send_drive(node, 0.0, 0.0);
  current_heading += M_PI;
  while (current_heading >= 2 * M_PI) current_heading -= 2 * M_PI;
}

void turn_90_CW(rclcpp::Node::SharedPtr node) {
  send_drive(node, 3.2, -3.2);
  rclcpp::sleep_for(std::chrono::seconds(2));
  send_drive(node, 0.0, 0.0);
  current_heading += M_PI_2;
  while (current_heading >= 2 * M_PI) current_heading -= 2 * M_PI;
}

void turn_90_CCW(rclcpp::Node::SharedPtr node) {
  send_drive(node, -3.2, 3.2);
  rclcpp::sleep_for(std::chrono::seconds(2));
  send_drive(node, 0.0, 0.0);
  current_heading -= M_PI_2;
  while (current_heading <  0)         current_heading += 2 * M_PI;
  while (current_heading >= 2 * M_PI)  current_heading -= 2 * M_PI;
}

void turn_to(rclcpp::Node::SharedPtr node, float target_heading) {
  float gamma = target_heading - current_heading;
  while (gamma >  M_PI) gamma -= 2 * M_PI;
  while (gamma < -M_PI) gamma += 2 * M_PI;
  if (std::fabs(gamma) < 1e-3f) return;

  RCLCPP_INFO(node->get_logger(), " already aligned.");
  float left_speed  = (gamma > 0) ?  3.2f : -3.2f;
  float right_speed = (gamma > 0) ? -3.2f :  3.2f;
  int   sleep_ms    = (int)std::round(4000.0 * std::fabs(gamma) / M_PI);

  send_drive(node, left_speed, right_speed);
  rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms));
  send_drive(node, 0.0, 0.0);

  current_heading = target_heading;
  while (current_heading <  0)         current_heading += 2 * M_PI;
  while (current_heading >= 2 * M_PI)  current_heading -= 2 * M_PI;
}

float calculate_speed(float dist, float dist_desired, float k) {
  return k * (dist_desired - dist);
}

void drive_distance(rclcpp::Node::SharedPtr node, float dist) {
  int time_ms = (int)(6000 * dist);
  send_drive(node, 5.4, 5.4);
  rclcpp::sleep_for(std::chrono::milliseconds(time_ms));
  send_drive(node, 0, 0);
}

//RANSAC types & constants 
struct Point {
  float x, y;
  int   beam_idx;
  int   scan_id;
};

struct Line {
  float alpha;
  float d;
  int   n_inliers;
};

const float INLIER_THRESHOLD = 0.008f;
const int   MIN_INLIERS      = 90;
const int   RANSAC_ITERS     = 2000;
const int   MAX_WALLS        = 5;
const int   MAX_BEAM_GAP     = 30;

// RANSAC helpers 
std::vector<Point> scan_to_points(const sensor_msgs::msg::LaserScan &scan,
                                  float rot_offset,
                                  float half_window,
                                  int   scan_id) {
  std::vector<Point> pts;
  pts.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); i++) {
    float rng = scan.ranges[i];
    if (!std::isfinite(rng)) continue;
    if (rng < scan.range_min || rng > scan.range_max) continue;
    float ang = scan.angle_min + i * scan.angle_increment;
    if (half_window > 0.0f && std::fabs(ang) > half_window) continue;
    float ang_world = ang + rot_offset;
    pts.push_back({rng * std::cos(ang_world), rng * std::sin(ang_world),
                  (int)i, scan_id});
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

    std::vector<int> beams_s0, beams_s1;
    for (int idx : inlier_idxs) {
      if (pts[idx].scan_id == 0) beams_s0.push_back(pts[idx].beam_idx);
      else                        beams_s1.push_back(pts[idx].beam_idx);
    }
    if (!inliers_contiguous(beams_s0)) continue;
    if (!inliers_contiguous(beams_s1)) continue;

    float dx    = b.x - a.x;
    float dy    = b.y - a.y;
    float alpha = std::atan2(dx, -dy);
    float d     = a.x * std::cos(alpha) + a.y * std::sin(alpha);
    if (d < 0) { d = -d; alpha += M_PI; }
    while (alpha <  0)         alpha += 2 * M_PI;
    while (alpha >= 2 * M_PI)  alpha -= 2 * M_PI;

    float min_tangential_dist = std::numeric_limits<float>::max();
    float tangent_x = -std::sin(alpha);
    float tangent_y =  std::cos(alpha);
    for (int idx : inlier_idxs) {
      float t = pts[idx].x * tangent_x + pts[idx].y * tangent_y;
      if (std::fabs(t) < min_tangential_dist)
        min_tangential_dist = std::fabs(t);
    }

    RCLCPP_INFO(node->get_logger(),
      "Wall Candidate // d=%.2f, alpha=%.2f, inliers=%d, tangential distance=%.4f m",
      d, alpha * 180.0f / M_PI, (int)inlier_idxs.size(), min_tangential_dist);

    if (min_tangential_dist > 0.01f) continue;

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

// your Line is {alpha, d}; in (a,b,c): a=cos(alpha), b=sin(alpha), c=-d
// point P satisfies a1*x+b1*y=d1 and a2*x+b2*y=d2
bool line_intersection(const Line &w1, const Line &w2, float &px, float &py) {
    float a1 = std::cos(w1.alpha), b1 = std::sin(w1.alpha), d1 = w1.d;
    float a2 = std::cos(w2.alpha), b2 = std::sin(w2.alpha), d2 = w2.d;
    float det = a1 * b2 - a2 * b1;
    if (std::fabs(det) < 1e-6f) return false;   // parallel
    px = ( b2 * d1 - b1 * d2) / det;
    py = (-a2 * d1 + a1 * d2) / det;
    return true;
}

void center_from_corner(const Line &w1, const Line &w2,
                        float px, float py, float &cx, float &cy) {
    float n1x = -std::cos(w1.alpha), n1y = -std::sin(w1.alpha); // toward origin
    float n2x = -std::cos(w2.alpha), n2y = -std::sin(w2.alpha);
    cx = px + HALF_CELL * n1x + HALF_CELL * n2x;
    cy = py + HALF_CELL * n1y + HALF_CELL * n2y;
}

// // align_to_wall 
// void align_to_wall(rclcpp::Node::SharedPtr node,
//                    const Line &w,
//                    const char *label,
//                    TurnType turn_type) {
//   float error = w.d - HALF_CELL;
//   float heading;
//   if (std::fabs(error) <= 0.03f) {
//     heading = M_PI - w.alpha;
//     RCLCPP_INFO(node->get_logger(), "%s: already aligned (d=%.2f).", label, w.d);
//     turn_to(node, heading);
//     return;
//   }

//   if (error > 0) {
//     heading = w.alpha;
//   } else {
//     heading = M_PI - w.alpha;
//   }
//   while (heading <  0)         heading += 2 * M_PI;
//   while (heading >= 2 * M_PI)  heading -= 2 * M_PI;

//   RCLCPP_INFO(node->get_logger(),
//     "%s: d=%.2f, error=%.2f, turn for %.1f deg, drive %.2f m",
//     label, w.d, error, heading * 180.0f / M_PI, std::fabs(error));

//   if      (turn_type == TurnType::TURN_FOR_A)     turn_to(node, heading);
//   else if (turn_type == TurnType::TURN_FOR_B_CW)  turn_90_CW(node);
//   else if (turn_type == TurnType::TURN_FOR_B_CCW) turn_90_CCW(node);

//   rclcpp::sleep_for(std::chrono::milliseconds(200));
//   drive_distance(node, std::fabs(error));
// }

// scan + wall detection (extracted helper) 
//
// Does the full two-scan RANSAC sequence from whatever position the robot is
// currently in and returns the detected walls (already delta-corrected and
// sorted by ascending distance).
//

std::vector<Line> do_scan_and_detect_walls(rclcpp::Node::SharedPtr node,
                                           rclcpp::Rate &rate,
                                           std::mt19937 &rng) {
  // --- scan 1: front 180\u00b0 ---
  current_heading = 0.0;
  rclcpp::sleep_for(std::chrono::seconds(2));
  scan_received = false;
  while (rclcpp::ok() && !scan_received) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  auto pts1 = scan_to_points(latest_scan, 0.0f, M_PI_2, 0);

  // --- turn 180\u00b0 and grab scan 2 ---
  turn_180(node);
  rclcpp::sleep_for(std::chrono::seconds(2));

  scan_received = false;
  while (rclcpp::ok() && !scan_received) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  auto pts2 = scan_to_points(latest_scan, M_PI, M_PI_2, 1);

  // --- merge ---
  std::vector<Point> pts;
  pts.reserve(pts1.size() + pts2.size());
  pts.insert(pts.end(), pts1.begin(), pts1.end());
  pts.insert(pts.end(), pts2.begin(), pts2.end());

  // --- RANSAC ---
  auto walls = extract_walls(node, pts, rng);

  // --- delta correction & sort ---
  for (auto &w : walls)
    w.d += std::fabs(delta * std::cos(w.alpha));

  std::sort(walls.begin(), walls.end(),
            [](const Line &a, const Line &b) { return a.d < b.d; });

  RCLCPP_INFO(node->get_logger(),
    "scan1: %zu pts, scan2: %zu pts, %zu walls detected",
    pts1.size(), pts2.size(), walls.size());
  for (size_t i = 0; i < walls.size(); i++)
    RCLCPP_INFO(node->get_logger(),
      "  wall %zu: alpha=%.1f deg, d=%.2f m, inliers=%d",
      i, walls[i].alpha * 180.0 / M_PI, walls[i].d, walls[i].n_inliers);

  return walls;
}

// alignment sequence (extracted helper)
//
// Given a freshly detected wall list, runs the full wall-A / wall-B
// centering and alignment manoeuvre.
//
void run_alignment_sequence(rclcpp::Node::SharedPtr node,
                            const std::vector<Line> &walls) {

  if (walls.size() < 2) {
    RCLCPP_WARN(node->get_logger(),
      "run_alignment_sequence: need at least 2 walls, got %zu — skipping.",
      walls.size());
    return;
  }

  // Pick wall A = nearest wall, then find the wall most perpendicular to it.
  const Line &wall_a = walls[0];
  const double PERP_TOLERANCE_RAD = 25.0 * M_PI / 180.0;

  int b_idx = -1;
  double best_perp_err = PERP_TOLERANCE_RAD;
  for (size_t i = 1; i < walls.size(); i++) {
    double diff = std::fabs(walls[i].alpha - wall_a.alpha);
    while (diff > M_PI) diff -= M_PI;                 // fold into [0, π)
    double perp_err = std::fabs(diff - M_PI / 2.0);   // distance from 90°
    if (perp_err < best_perp_err) {
      best_perp_err = perp_err;
      b_idx = (int)i;
    }
  }

  if (b_idx < 0) {
    RCLCPP_WARN(node->get_logger(),
      "run_alignment_sequence: no perpendicular wall found — skipping.");
    return;
  }
  const Line &wall_b = walls[b_idx];

  // Corner = intersection of the two perpendicular walls.
  float px, py;
  if (!line_intersection(wall_a, wall_b, px, py)) {
    RCLCPP_WARN(node->get_logger(),
      "run_alignment_sequence: walls parallel, no intersection — skipping.");
    return;
  }

  // Cell center = step half a cell inward along each wall normal from the corner.
  float cx, cy;
  center_from_corner(wall_a, wall_b, px, py, cx, cy);

  RCLCPP_INFO(node->get_logger(),
    "corner=(%.2f, %.2f), center=(%.2f, %.2f)", px, py, cx, cy);

  // Turn toward the center and drive there.
  float heading_to_center = std::atan2(cy, cx);   // robot frame, x forward
  float dist_to_center    = std::hypot(cx, cy);

  turn_to(node, heading_to_center);
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  drive_distance(node, dist_to_center);

  RCLCPP_INFO(node->get_logger(), "alignment sequence done");
}

// main 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("align");

  auto sensor_sub = node->create_subscription<create_fundamentals::msg::SensorPacket>(
    "sensor_packet", 10,
    [&](const create_fundamentals::msg::SensorPacket &msg) { sensor_callback(msg); });

  auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", 10,
    [&](const sensor_msgs::msg::LaserScan &msg) { laser_callback(msg); });

  rclcpp::Rate rate(100);
  std::random_device rd;
  std::mt19937 rng(rd());

  initialize_sounds();

  RCLCPP_INFO(node->get_logger(), "=== initial scan ===");
  auto walls = do_scan_and_detect_walls(node, rate, rng);

  bool any_close = false;
  for (const auto &w : walls) {
    if (w.d <= HALF_CELL){
      any_close = true; 
      break; 
    }
  }

  if (!any_close) {
  size_t nearest_idx = 0;
  for (size_t i = 1; i < walls.size(); i++)
    if (walls[i].d < walls[nearest_idx].d) nearest_idx = i;

  const Line &nearest = walls[nearest_idx];
  float drive_dist = nearest.d - 0.35f;
  float a_deg = nearest.alpha * 180.0f / M_PI;

  RCLCPP_INFO(node->get_logger(),
    "edge case: nearest alpha=%.1f deg, d=%.2f m, drive %.2f m",
    a_deg, nearest.d, drive_dist);

  if      (a_deg < 60.0f  || a_deg >= 300.0f) turn_180(node);
  else if (a_deg < 120.0f)                    turn_90_CCW(node);
  else if (a_deg < 240.0f)                    { }
  else                                         turn_90_CW(node);
    
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  drive_distance(node, drive_dist);
  
  RCLCPP_INFO(node->get_logger(), "=== re-scan after repositioning ===");
  walls.clear();
  walls = do_scan_and_detect_walls(node, rate, rng);
  } else {
    RCLCPP_INFO(node->get_logger(),
      "=== at least one wall <= 40 cm -> no repositioning needed ===");
  }

  RCLCPP_INFO(node->get_logger(), "=== running alignment sequence ===");
  run_alignment_sequence(node, walls);

  play_success_sound();
  rclcpp::sleep_for(std::chrono::seconds(3));
  play_failure_sound();

  rclcpp::shutdown();
  return 0;
}
