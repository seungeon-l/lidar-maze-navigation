#include "create_fundamentals/msg/sensor_packet.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "lidar_maze_navigation/msg/exact_pose.hpp"
#include "lidar_maze_navigation/msg/grid.hpp"
#include "lidar_maze_navigation/msg/motion.hpp"
#include "lidar_maze_navigation/msg/pose.hpp"
#include "lidar_maze_navigation/msg/ransac.hpp"
#include "lidar_maze_navigation/srv/dijkstra.hpp"
#include "lidar_maze_navigation/srv/execute_plan.hpp"
#include "lidar_maze_navigation/srv/move_to_position.hpp"
#include "lidar_maze_navigation/srv/potential_field.hpp"
#include "localization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

// ================================================================================
// GLOBAL VARIABLES
// ================================================================================
const float r = 0.032;
const float b = 0.261;
const float delta = 0.128;
const float circle_rad = 0.4;
const float cell_length = 0.8;
const float HALF_CELL = 0.4;

const float speed_turn = 5;
const float arc_turn = circle_rad * M_PI_2;

float encoder_left = 0.0;
float encoder_right = 0.0;
float current_cmd_left = 0.0;
float current_cmd_right = 0.0;
std::chrono::steady_clock::time_point last_time =
    std::chrono::steady_clock::now();

// bool TURN_INCOMING = false;
// bool JUST_TURNED = false;
// bool FIRST_COMMAND = false;
// bool LAST_COMMAND = false;
// bool TURN_180 = false;

struct Point {
  float x;
  float y;
};

struct WallInfo {
  bool found;
  float distance;
};

// struct Straight {
//     int start;
//     int nr_cells;
// };

// std::vector<int> plan;

sensor_msgs::msg::LaserScan latest_scan;
bool scan_received = false;

std::vector<lidar_maze_navigation::msg::Line> walls;

// int state = 1;

lidar_maze_navigation::msg::Pose current_pose;
bool pose_received = false;
bool current_pose_valid = false;

bool initial_pose_set = false;
bool plan_running = false;
float expected_x = 0.0;
float expected_y = 0.0;
float expected_theta = 0.0;

bool just_localized = false;

bool pickup = false;

std::vector<lidar_maze_navigation::msg::Row> global_map;
bool map_received = false;
bool is_lost = false;

std::vector<std::vector<std::vector<std::pair<float, float>>>>
    potential_field_map;
bool field_ready = false;
int field_row = 0;
int field_col = 0;

enum class WanderState { NOT_AVOIDING, TURNING_LEFT, TURNING_RIGHT };
WanderState wander_state = WanderState::NOT_AVOIDING;

std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> reset_loc_client;
std::shared_ptr<rclcpp::Client<create_fundamentals::srv::DiffDrive>>
    diff_drive_client;

rclcpp::Publisher<lidar_maze_navigation::msg::Pose>::SharedPtr pose_pub;

lidar_maze_navigation::msg::Motion mmsg;
rclcpp::Publisher<lidar_maze_navigation::msg::Motion>::SharedPtr motion_pub;
rclcpp::Publisher<lidar_maze_navigation::msg::Pose>::SharedPtr teleport_pub;

std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

// ================================================================================
// CALLBACKS
// ================================================================================

void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
  latest_scan = msg;
  scan_received = true;
}

void ransac_callback(const lidar_maze_navigation::msg::Ransac &msg) {
  walls = msg.walls;
}

void pose_callback(const lidar_maze_navigation::msg::Pose &msg) {
  current_pose = msg;
  current_pose_valid = true;
  pose_received = true;

  if (!initial_pose_set) {
    expected_x = msg.column * cell_length + HALF_CELL;
    expected_y = msg.row * cell_length + HALF_CELL;
    float orient_angles[4] = {0.0f, 3.0f * M_PI_2, M_PI, M_PI_2};
    expected_theta = orient_angles[msg.orientation];
    initial_pose_set = true;
  } else if (just_localized && !plan_running) {
    float orient_angles[4] = {0.0f, 3.0f * M_PI_2, M_PI, M_PI_2};
    expected_theta = orient_angles[msg.orientation];
    expected_x = msg.column * cell_length + HALF_CELL;
    expected_y = msg.row * cell_length + HALF_CELL;
    just_localized = false;
  }
}

void map_callback(const lidar_maze_navigation::msg::Grid &msg) {
  global_map = msg.rows;
  map_received = true;
}

void exact_pose_callback(const lidar_maze_navigation::msg::ExactPose &msg) {
  field_row = msg.current_row;
  field_col = msg.current_col;
}

// ================================================================================
// ENCODERS
// ================================================================================

void update_encoder() {
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt = now - last_time;
  last_time = now;

  float d_left_cmd = current_cmd_left * dt.count();
  float d_right_cmd = current_cmd_right * dt.count();

  encoder_left += d_left_cmd;
  encoder_right += d_right_cmd;

  float d_left_dist = d_left_cmd * r;
  float d_right_dist = d_right_cmd * r;

  float d_center = (d_left_dist + d_right_dist) / 2.0;
  float d_theta = (d_right_dist - d_left_dist) / b;
  float d_theta_map = -d_theta;

  expected_x += d_center * cos(expected_theta + d_theta_map / 2.0);
  expected_y += d_center * sin(expected_theta + d_theta_map / 2.0);
  expected_theta += d_theta_map;

  mmsg.d_dist += d_center;
  mmsg.d_angle += d_theta;
  motion_pub->publish(mmsg);
}

void reset_encoder() {
  encoder_left = 0.0;
  encoder_right = 0.0;
  last_time = std::chrono::steady_clock::now();
}

// ================================================================================
// HELPERS
// ================================================================================

// sound helpers
void initialize_sounds() {
  std::string store_success =
      "ros2 service call /store_song create_fundamentals/srv/StoreSong "
      "\"{number: 2, song: [72, 16, 76, 16, 79, 16, 84, 32]}\"";
  // the initial setup sound when running driver.py is,
  // 64, 16, 69, 16, 74, 16, 72, 40, 69, 60

  std::string store_failure =
      "ros2 service call /store_song create_fundamentals/srv/StoreSong "
      "\"{number: 3, song: [60, 32, 55, 48]}\"";

  // std::cout << "Attempting to store success sound..." << std::endl;
  std::system(store_success.c_str());

  // std::cout << "Attempting to store failure sound..." << std::endl;
  std::system(store_failure.c_str());
}

void play_warning_sound() {
  std::system("ros2 service call /play_song create_fundamentals/srv/PlaySong "
              "\"{number: 3}\"");
}

void wait_for_helicopter() {
  // TO DO: change song
  std::system("ros2 service call /play_song create_fundamentals/srv/PlaySong "
              "\"{number: 2}\"");
}

void get_gold() {
  // TODO: make sound 5 seconds long
  std::system("ros2 service call /play_song create_fundamentals/srv/PlaySong "
              "\"{number: 2}\"");
}

void send_drive(std::shared_ptr<rclcpp::Node> node, float left, float right) {
  current_cmd_left = left;
  current_cmd_right = right;

  auto request =
      std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
  request->left = left;
  request->right = right;
  diff_drive_client->async_send_request(request);
}

// bool check_if_lost(
//     std::shared_ptr<rclcpp::Node> node,
//     std::shared_ptr<lidar_maze_navigation::srv::MoveToPosition::Response>
//         response,
//     const std::vector<std::pair<float, float>> &path_world, float expected_x,
//     float expected_y) {
//   // calculate robot's distance to the nearest subgoal
//   float min_path_dist = std::numeric_limits<float>::max();
//   for (auto &[wx, wy] : path_world) {
//     float dx = expected_x - wx;
//     float dy = expected_y - wy;
//     float d = std::sqrt(dx * dx + dy * dy);
//     if (d < min_path_dist)
//       min_path_dist = d;
//   }

//   if (min_path_dist > 0.45f) {
//     send_drive(node, 0.0f, 0.0f);
//     rclcpp::sleep_for(std::chrono::milliseconds(200));
//     play_warning_sound();
//     RCLCPP_ERROR(node->get_logger(),
//                  "Lost! Odometry (%.2f, %.2f) is %.2fm from nearest path
//                  cell.", expected_x, expected_y, min_path_dist);
//     response->success = false;
//     plan_running = false;
//     if (reset_loc_client->wait_for_service(std::chrono::seconds(2))) {
//       reset_loc_client->async_send_request(
//           std::make_shared<std_srvs::srv::Trigger::Request>());
//     }
//     // true - robot is lost
//     return true;
//   }

//   // false - robot is not lost
//   return false;
// }


int theta_to_orientation(float theta) {
  float angles[4] = {0.0f, 3.0f * M_PI_2, M_PI, M_PI_2}; // R,UP,L,DOWN
  int best = 0; float best_diff = 1e9f;
  for (int i = 0; i < 4; i++) {
    float d = std::fabs(theta - angles[i]);
    if (d > M_PI) d = 2 * M_PI - d;
    if (d < best_diff) { best_diff = d; best = i; }
  }
  return best;
}

WallInfo wall_check(float center_degree, float threshold = 0.6f) {
  WallInfo result = {false, -1.0f};

  if (!scan_received)
    return result;

  int total = latest_scan.ranges.size();
  if (total == 0)
    return result;

  float lookup_degree = 40.0f;
  float lookup_index_half = total * (lookup_degree / 2.0f / 240.0f);

  int scan_center = (int)(total * (center_degree / 240.0f));
  int scan_start = std::max(0, scan_center - (int)lookup_index_half);
  int scan_end = std::min(total - 1, scan_center + (int)lookup_index_half);

  int total_rays_in_cone = scan_end - scan_start + 1;
  if (total_rays_in_cone <= 0)
    return result;

  int hit_count = 0;
  float min_dist = std::numeric_limits<float>::infinity();

  for (int i = scan_start; i <= scan_end; i++) {
    float r = latest_scan.ranges[i];

    if (std::isfinite(r) && r > latest_scan.range_min &&
        r < latest_scan.range_max) {
      if (r < threshold) {
        hit_count++;
        if (r < min_dist) {
          min_dist = r;
        }
      }
    }
  }

  float hit_ratio = (float)hit_count / (float)total_rays_in_cone;

  if (hit_ratio > 0.70f) {
    result.found = true;
    result.distance = min_dist;
  }

  return result;
}

bool side_wall_parallel(float tol_deg = 5.0f) {
  float tol = tol_deg * M_PI / 180.0f;
  for (const auto &w : walls) {
    float diff_left = std::fabs(w.angle - M_PI_2);
    float diff_right = std::fabs(w.angle - 3.0f * M_PI_2);
    if (diff_left < tol || diff_right < tol) {
      return true;
    }
  }
  return false;
}

float front_wall_distance(std::shared_ptr<rclcpp::Node> node,
                          float tol_deg = 20.0f) {
  float tol = tol_deg * M_PI / 180.0f;
  float min_dist = -1.0f;
  for (const auto &w : walls) {
    float diff_zero = std::fabs(w.angle);
    float diff_2pi = std::fabs(w.angle - 2.0f * M_PI);
    if (diff_zero < tol || diff_2pi < tol) {
      if (min_dist < 0.0f || w.distance < min_dist) {
        min_dist = w.distance;
      }
    }
  }
  return min_dist;
}

float calculate_speed(float dist, float dist_desired, float k) {
  return k * (dist_desired - dist);
}

// minimum range in a +-15 deg cone straight ahead, from the laser scan.
// returns a big number if no scan yet.
float front_min_range() {
  if (!scan_received)
    return 1e9f;
  int total = latest_scan.ranges.size();
  if (total == 0)
    return 1e9f;
  float step = 240.0f / total; // deg per index
  int half = (int)(8.0f / step);
  int c = total / 2;
  float best = 1e9f;
  for (int i = c - half; i <= c + half && i < total; i++) {
    if (i < 0)
      continue;
    float v = latest_scan.ranges[i];
    if (std::isfinite(v) && v > 0.02f && v < best)
      best = v;
  }
  return best;
}

// heading error just for 1st in-place turn if needed
float heading_err_to_wp(std::vector<std::pair<float, float>> path_world,
                        size_t wp) {
  float tx0 = path_world[wp].first - expected_x;
  float ty0 = path_world[wp].second - expected_y;
  float d = std::atan2(ty0, tx0) - expected_theta;

  while (d > M_PI)
    d -= 2 * M_PI;
  while (d < -M_PI)
    d += 2 * M_PI;

  return d;
}

void recover_from_wall(std::shared_ptr<rclcpp::Node> node) {
  // 1. back up ~20 cm
  RCLCPP_INFO(node->get_logger(), "recovery: backing up 20cm");
  send_drive(node, -5.2f, -5.2f);
  rclcpp::sleep_for(std::chrono::milliseconds(1200));
  send_drive(node, 0.0f, 0.0f);
  rclcpp::sleep_for(std::chrono::milliseconds(150));

  // 2. align to the nearest cardinal heading {0, 3\u03c0/2, \u03c0, \u03c0/2}
  float cardinals[4] = {0.0f, 3.0f * (float)M_PI_2, (float)M_PI, (float)M_PI_2};
  float target = cardinals[0];
  float best_diff = 1e9f;
  for (int i = 0; i < 4; i++) {
    float d = std::fabs(expected_theta - cardinals[i]);
    if (d > M_PI) d = 2 * M_PI - d;
    if (d < best_diff) { best_diff = d; target = cardinals[i]; }
  }

  float err = target - expected_theta;
  while (err > M_PI)  err -= 2 * M_PI;
  while (err < -M_PI) err += 2 * M_PI;

  RCLCPP_INFO(node->get_logger(), "recovery: aligning to %.2f (err %.2f)", target, err);
  float w = std::copysign(2.5f, err);
  send_drive(node, -w, w);              // in-place rotation toward target
  int ticks = 0;
  while (std::fabs(err) > 0.05f && rclcpp::ok() && ticks++ < 400) {
    update_encoder();
    err = target - expected_theta;
    while (err > M_PI)  err -= 2 * M_PI;
    while (err < -M_PI) err += 2 * M_PI;
    rate.sleep();
  }
  send_drive(node, 0.0f, 0.0f);
  rclcpp::sleep_for(std::chrono::milliseconds(150));
}

void follow_plan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>
        request,
    std::shared_ptr<std_srvs::srv::Trigger::Response>
        response,
    std::shared_ptr<rclcpp::Node> node) {

  RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
  RCLCPP_INFO(node->get_logger(), "Start tracking!!!");
  RCLCPP_INFO(node->get_logger(), "----------------------------------------------");

  // Big Dijkstra service call: get optimal order to visit gold locations in +
  // closest pickup spot (/gold_order)
  auto go_client =
      node->create_client<lidar_maze_navigation::srv::Dijkstra>("gold_order");
  while (!go_client->wait_for_service(std::chrono::seconds(1)))
    RCLCPP_INFO(node->get_logger(), "waiting for big dijkstra service...");
  auto go_req =
      std::make_shared<lidar_maze_navigation::srv::Dijkstra::Request>();
  // service doesn't need any request, set to 0 so it's not empty
  go_req->goal_row = 0;
  go_req->goal_column = 0;
  auto go_future = go_client->async_send_request(go_req);
  rclcpp::Rate wait_rate(100);
  while (go_future.wait_for(std::chrono::milliseconds(10)) !=
             std::future_status::ready &&
         rclcpp::ok()) {
    wait_rate.sleep();
  }
  auto go_resp = go_future.get();

  std::vector<std::pair<int, int>> gold_cells;
  for (size_t i = 0; i < go_resp->path_rows.size(); i++) {
    gold_cells.push_back({go_resp->path_rows[i], go_resp->path_columns[i]});
    RCLCPP_INFO(node->get_logger(), "gold nr #%i: row=%i, column=%i", i, go_resp->path_rows[i], go_resp->path_columns[i]);

  }

  // export gold locations for plotter
  std::ofstream gold_file(
      "src/lidar_maze_navigation/gold_locations.csv");
  gold_file << "row,col\n";
  for (const auto &gc : gold_cells) {
    gold_file << gc.first << "," << gc.second << "\n";
  }
  gold_file.close();

  // client for little dijkstra, used later in the while loop
  auto sp_client = node->create_client<lidar_maze_navigation::srv::Dijkstra>(
      "shortest_path");

  // potential field service call: (request is empty \u2014 field is built from
  // the map)
  auto pf_client =
      node->create_client<lidar_maze_navigation::srv::PotentialField>(
          "potential_field");
  while (!pf_client->wait_for_service(std::chrono::seconds(1)))
    RCLCPP_INFO(node->get_logger(), "waiting for potential_field service...");

  auto pf_req =
      std::make_shared<lidar_maze_navigation::srv::PotentialField::Request>();
  auto pf_future = pf_client->async_send_request(pf_req);
  while (pf_future.wait_for(std::chrono::milliseconds(10)) !=
             std::future_status::ready &&
         rclcpp::ok())
    wait_rate.sleep();
  auto pf_resp = pf_future.get();

  // reconstruct 2D map from response 
  int rows = pf_resp->rows, cols = pf_resp->cols;
  potential_field_map.assign(
      rows, std::vector<std::vector<std::pair<float, float>>>(
                cols, std::vector<std::pair<float, float>>(2)));
  for (int f = 0; f < 2; f++)
    for (int r = 0; r < rows; r++)
      for (int c = 0; c < cols; c++) {
        potential_field_map[r][c][f].first =
            pf_resp->gx[f * rows * cols + r * cols + c];
        potential_field_map[r][c][f].second =
            pf_resp->gy[f * rows * cols + r * cols + c];
      }

  field_ready = true;
  bool pickup_spot = false;

  // iterate through the gold cells in the given order:
  // 1. get shortest path towards the next gold
  // 2. folloe plan on shortest path
  // 3. sing for money
  for (int g = 0; g < gold_cells.size(); g++) {
    // task1: takes one goal from terminal - now we get goals from the gold and
    // pickup txt files int goal_row = request->row; int goal_column =
    // request->column;

    std::pair<int, int> goal = gold_cells[g];
    int goal_row = goal.first;
    int goal_column = goal.second;

    // export current goal for plotter
    std::ofstream goal_file(
        "src/lidar_maze_navigation/goal.csv");
    goal_file << "row,col\n" << goal_row << "," << goal_column << "\n";
    goal_file.close();

    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
    RCLCPP_INFO(node->get_logger(), "next goal: row=%i, column=%i", goal_row, goal_column);
    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");

    if (g == gold_cells.size() - 1) {
      pickup_spot = true;
    }

    // Small Dijkstra service call: get shortest path towards the next goal
    // (/shortest_path)
    while (!sp_client->wait_for_service(std::chrono::seconds(1)))
      RCLCPP_INFO(node->get_logger(), "waiting for small dijkstra service...");
    auto sp_req =
        std::make_shared<lidar_maze_navigation::srv::Dijkstra::Request>();
    sp_req->goal_row = goal_row;
    sp_req->goal_column = goal_column;
    auto sp_future = sp_client->async_send_request(sp_req);
    rclcpp::Rate wait_rate(100);
    while (sp_future.wait_for(std::chrono::milliseconds(10)) !=
               std::future_status::ready &&
           rclcpp::ok()) {
      wait_rate.sleep();
    }
    auto sp_resp = sp_future.get();

    std::vector<std::pair<int, int>> path_cells;
    std::vector<bool> path_turns;
    for (size_t i = 0; i < sp_resp->path_rows.size(); i++) {
      path_cells.push_back({sp_resp->path_rows[i], sp_resp->path_columns[i]});
      path_turns.push_back(sp_resp->path_turns[i]); // fix corner wp
    }

    // print path
    std::ofstream pf_file(
        "src/lidar_maze_navigation/path.csv");
    pf_file << "row,col\n";
    for (int i = 0; i < path_cells.size(); i++) {
      pf_file << path_cells[i].first << "," << path_cells[i].second << "\n";
    }
    pf_file.close();

    size_t wp = 1; // waypoint 0 is the start cell center; aim at the next one
    int danger_count = 0;

    float goal_x = goal_column * cell_length + HALF_CELL;
    float goal_y = goal_row * cell_length + HALF_CELL;

    bool lost;

    // path in metric coordinates
    std::vector<std::pair<float, float>> path_world;
    // for (auto &[pr, pc] : path_cells) {cells
    //   path_world.push_back( 
    //       {pc * cell_length + HALF_CELL, pr * cell_length + HALF_CELL});
    // }

    // fix corner wp
    path_world.push_back(
      {path_world[0].second * cell_length + HALF_CELL, path_world[0].first * cell_length + HALF_CELL});
    
      for(int p = 1; p < path_cells.size(); p++) {
      float prev_c = path_world[p-1].first;
      float prev_r = path_world[p-1].second;
      float curr_c = path_world[p].second * cell_length + HALF_CELL;
      float curr_r = path_world[p].first * cell_length + HALF_CELL;

      if(path_turns[p] == true) {
        float delta_c = curr_c - prev_c;
        float delta_r = curr_r - prev_r;

        curr_c -= delta_c * 1/8;
        curr_r -= delta_r * 1/8;
      }
      
      path_world.push_back({curr_c, curr_r});     
       
    }

    while (!diff_drive_client->wait_for_service(std::chrono::seconds(1)) &&
           rclcpp::ok())
      RCLCPP_INFO(node->get_logger(), "waiting for diff_drive service...");

    // variables for tracking the traveled distance
    float dist_traveled = 0.0f;
    float prev_x = expected_x;
    float prev_y = expected_y;
    int csv_counter = 0;

    bool pivoting = false;
    const float PIVOT_ENTER = 2.0f; // ~115\u00b0, only 180 turns should cause pivoting
    const float PIVOT_EXIT = 0.35f; // ~20\u00b0, stay pivoting until well aligned

    // new smooth: handle first 90 or 180 degree turn before starting the plan
    // calculate heading error for first step
    float err0 = heading_err_to_wp(path_world, wp);

    plan_running = true;

    // turn in place before starting the plan
    rclcpp::Rate align_rate(100);
    while (std::fabs(err0) > 1.5f && rclcpp::ok()) {
      update_encoder();
      float w0 = std::copysign(2.5f, err0);
      send_drive(node, w0, -w0);
      align_rate.sleep();
      err0 = heading_err_to_wp(path_world, wp);
    }
    send_drive(node, 0.0f, 0.0f);

    // start the plan: follow shortest path to the next goal
    rclcpp::Rate rate(100);
    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
    RCLCPP_INFO(node->get_logger(), "starting plan!!");
    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");

    while (rclcpp::ok()) {
      update_encoder();

      if (is_lost) {
        send_drive(node, 0.0f, 0.0f);
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        play_warning_sound();

        response->success = false;
        plan_running = false;
        RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
        RCLCPP_INFO(node->get_logger(), "i'm lost!!");
        RCLCPP_INFO(node->get_logger(), "----------------------------------------------");

        if (reset_loc_client->wait_for_service(std::chrono::seconds(2))) {
          reset_loc_client->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>());
        }
        // break;
        goto done;
      }

      float tx = path_world[wp].first - expected_x;
      float ty = path_world[wp].second - expected_y;
      float td = std::sqrt(tx * tx + ty * ty);

      bool on_final = (wp + 1 >= path_world.size());

      if (!on_final) {
        if (td < 0.18f) {
          wp++;
          continue;
        }
      } else {
        // measure AND steer from the center on final approach
        float cx = expected_x - delta * std::cos(expected_theta);
        float cy = expected_y - delta * std::sin(expected_theta);
        tx = path_world[wp].first - cx; // overwrite tx,ty with center-frame
        ty = path_world[wp].second - cy;
        td = std::sqrt(tx * tx + ty * ty);
        float along =
            tx * std::cos(expected_theta) + ty * std::sin(expected_theta);
        if (along <= 0.0f || td < 0.06f) {
          break;
        }
      }

      float ax = (td > 1e-6f) ? tx / td : 0.0f;
      float ay = (td > 1e-6f) ? ty / td : 0.0f;

      float vis_x = on_final ? (expected_x - delta * std::cos(expected_theta)) : expected_x;
      float vis_y = on_final ? (expected_y - delta * std::sin(expected_theta)) : expected_y;
      std::ofstream attr_file(
          "src/lidar_maze_navigation/attractive.csv");
      attr_file << "x,y,ax,ay\n";
      attr_file << vis_x << "," << vis_y << "," << ax << "," << ay << "\n";
      attr_file.close();

      float gx = ax;
      float gy = ay;
      if (field_ready && field_row >= 0 &&
          field_row < (int)potential_field_map.size() && field_col >= 0 &&
          field_col < (int)potential_field_map[field_row].size()) {

        float rx = potential_field_map[field_row][field_col][0].first;
        float ry = potential_field_map[field_row][field_col][0].second;

        float pre_err = std::atan2(gy, gx) - expected_theta;
        while (pre_err > M_PI)
          pre_err -= 2 * M_PI;
        while (pre_err < -M_PI)
          pre_err += 2 * M_PI;
        if (std::fabs(pre_err) < 1.3f) {
          gx += rx;
          gy += ry;
        }
      }

      //   heading controller
      float desired = std::atan2(gy, gx);
      float err = desired - expected_theta;
      while (err > M_PI)
        err -= 2 * M_PI;
      while (err < -M_PI)
        err += 2 * M_PI;

      if (on_final) {
        desired = std::atan2(ay, ax);
        err = desired - expected_theta;
        while (err > M_PI)
          err -= 2 * M_PI;
        while (err < -M_PI)
          err += 2 * M_PI;
        if (std::fabs(err) < 0.17f)
          err = 0.0f;
      }

      //   front safety guard: never drive into a wall
      float left, right;
      bool driving_forward = (std::fabs(err) <= 0.6f);  
      float fd = front_min_range();
      if (fd < 0.05f && driving_forward) {
         if (++danger_count > 100) {
          send_drive(node, 0.0f, 0.0f);
          RCLCPP_ERROR( node->get_logger(), "front blocked: fd=%.3f err=%.2f drive=%d", fd, err, (int)driving_forward);

          recover_from_wall(node);
          
          response->success = false;
          play_warning_sound();
          plan_running = false;
          if (reset_loc_client->wait_for_service(std::chrono::seconds(2))) {
            reset_loc_client->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
          }
          // 19june
          // not break, go to!!!!
          // break;
          goto done;
        }
        send_drive(node, 0.0f, 0.0f);
        rate.sleep();
        continue;
      }
      danger_count = 0; 

      // normal motion: scale forward speed by alignment
      float v = 6.0f * std::max(0.0f, std::cos(err));
      if (fd < 0.35f)
        v = std::min(v, 2.5f); // creep when close
      float w = std::clamp(5.0f * err, -5.0f, 5.0f);


      if (!on_final) {
        if (!pivoting && std::fabs(err) > PIVOT_ENTER)
          pivoting = true;
        else if (pivoting && std::fabs(err) < PIVOT_EXIT)
          pivoting = false;
      } else {
        pivoting = false; // never pivot on final approach
      }

      if (pivoting) {
        left = std::copysign(2.5f, w);
        right = -std::copysign(2.5f, w);
      } else {
        left = std::clamp(v + w, 1.0f, 9.0f);
        right = std::clamp(v - w, 1.0f, 9.0f);
      }

      // left  = std::clamp(v + w, 1.0f, 9.0f);
      // right = std::clamp(v - w, 1.0f, 9.0f);
      send_drive(node, left, right);

      rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
    RCLCPP_INFO(node->get_logger(), "finished plan!!!!!!!!!!");
    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");

    plan_running = false;
    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    lidar_maze_navigation::msg::Pose tp;
    tp.row = goal_row;
    tp.column = goal_column;
    tp.orientation = theta_to_orientation(expected_theta);
    teleport_pub->publish(tp);
    RCLCPP_INFO(node->get_logger(),
      "teleport cloud -> row=%d col=%d orient=%d", tp.row, tp.column, tp.orientation);
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // sing
    if (pickup_spot) {
      RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
      RCLCPP_INFO(node->get_logger(), "waiting for helicopter!!!");
      RCLCPP_INFO(node->get_logger(), "----------------------------------------------");

      wait_for_helicopter();
    } else {
      RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
      RCLCPP_INFO(node->get_logger(), "getting gold!!!");
      RCLCPP_INFO(node->get_logger(), "----------------------------------------------");

      get_gold();
    }
  }

  done:
  {
    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
    RCLCPP_INFO(node->get_logger(), "stopped gold search!");
    RCLCPP_INFO(node->get_logger(), "----------------------------------------------");
  }
}

// ================================================================================
// MAIN
// ================================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  just_localized = true;

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("tracking_node");
  diff_drive_client =
      node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive");

  pose_pub =
      node->create_publisher<lidar_maze_navigation::msg::Pose>("pose", 10);
  motion_pub = node->create_publisher<lidar_maze_navigation::msg::Motion>(
      "motion_delta", 50);

  teleport_pub =
      node->create_publisher<lidar_maze_navigation::msg::Pose>("teleport_to", 10);

  auto service_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sensor_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto ransac_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pose_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto map_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto exact_pose_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lost_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Reset Localization Client
  reset_loc_client =
      node->create_client<std_srvs::srv::Trigger>("reset_global_localization");

  // Execute Plan Service
  auto plan_service = node->create_service<std_srvs::srv::Trigger>(
      "follow_plan",
      [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        follow_plan(request, response, node);
      },
      rmw_qos_profile_services_default, service_cbg);

  // Laser Subscriber
  rclcpp::SubscriptionOptions sensor_opts;
  sensor_opts.callback_group = sensor_cbg;
  auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", 10,
      [&](const sensor_msgs::msg::LaserScan &msg) { laser_callback(msg); },
      sensor_opts);

  // RANSAC Subscriber
  rclcpp::SubscriptionOptions ransac_opts;
  ransac_opts.callback_group = ransac_cbg;
  auto ransac_sub =
      node->create_subscription<lidar_maze_navigation::msg::Ransac>(
          "ransac", 10,
          [&](const lidar_maze_navigation::msg::Ransac &msg) {
            ransac_callback(msg);
          },
          ransac_opts);

  // Pose Subscriber
  rclcpp::SubscriptionOptions pose_opts;
  pose_opts.callback_group = pose_cbg;
  auto pose_sub = node->create_subscription<lidar_maze_navigation::msg::Pose>(
      "pose", 10,
      [&](const lidar_maze_navigation::msg::Pose &msg) { pose_callback(msg); },
      pose_opts);

  // Map Subscriber
  rclcpp::SubscriptionOptions map_opts;
  map_opts.callback_group = map_cbg;
  auto map_sub = node->create_subscription<lidar_maze_navigation::msg::Grid>(
      "map", 10,
      [&](const lidar_maze_navigation::msg::Grid &msg) { map_callback(msg); },
      map_opts);

  // Exact Pose subscriber
  rclcpp::SubscriptionOptions exact_pose_opts;
  exact_pose_opts.callback_group = exact_pose_cbg;
  auto exact_pose_sub =
      node->create_subscription<lidar_maze_navigation::msg::ExactPose>(
          "exact_pose", 10,
          [&](const lidar_maze_navigation::msg::ExactPose &msg) {
            exact_pose_callback(msg);
          },
          exact_pose_opts);

  rclcpp::SubscriptionOptions lost_opts;
  lost_opts.callback_group = lost_cbg;
  auto lost_sub = node->create_subscription<std_msgs::msg::Bool>(
      "localization_status_lost", 10,
      [&](const std_msgs::msg::Bool &msg) { is_lost = msg.data; }, lost_opts);

  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
