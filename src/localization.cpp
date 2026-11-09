#include "localization.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "lidar_maze_navigation/msg/cell.hpp"
#include "lidar_maze_navigation/msg/exact_pose.hpp"
#include "lidar_maze_navigation/msg/grid.hpp"
#include "lidar_maze_navigation/msg/motion.hpp"
#include "lidar_maze_navigation/msg/pose.hpp"
#include "lidar_maze_navigation/msg/ransac.hpp"
#include "lidar_maze_navigation/msg/row.hpp"
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
#include <mutex>
#include <random>
#include <string>
#include <vector>

// Robot geometry
const float r = 0.032;
const float b = 0.261;
const float delta = 0.128;

const float circle_rad = 0.4; // radius of turning circle; 0.15 < circle_rad <
                              // 0.55, so that the robot doesn't hit any walls
const float cell_length = 0.8;
const float HALF_CELL = 0.4;
const float mini_cell_length = cell_length / 10.0f;

const float speed_turn = 5;
const float arc_turn = circle_rad * M_PI_2;

// our encoder sim
float encoder_left = 0.0;
float encoder_right = 0.0;
float current_cmd_left = 0.0;
float current_cmd_right = 0.0;
std::chrono::steady_clock::time_point last_time =
    std::chrono::steady_clock::now();
// echte encoders

float real_encoder_left = 0.0;
float real_encoder_right = 0.0;
float encoder_left_baseline = 0.0;
float encoder_right_baseline = 0.0;

int consecutive_turns = 0; // how many turns since we last drove
int last_turn_dir = 0;     // -1 = CCW, +1 = CW, 0 = none

float prev_mean_x = 1e9f;
float prev_mean_y = 1e9f;
int stable_count = 0;

sensor_msgs::msg::LaserScan latest_scan;
bool motion_received = false;
bool scan_received = false;

std::mutex motion_mutex;

float pending_d_dist = 0.0f;
float pending_d_angle = 0.0f;
float last_seen_total_dist = 0.0f;
float last_seen_total_angle = 0.0f;
bool first_motion_msg = true;

std::vector<lidar_maze_navigation::msg::Line> walls;

std::vector<lidar_maze_navigation::msg::Row> map;
std::vector<std::vector<int>> wall_map; // 10x10 mini cells in each big cell

// Particle struct is defined in localization.hpp
std::vector<Particle> particle_map;

std::vector<std::vector<bool>> open_cell;

std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> follow_plan_client;

rclcpp::Publisher<lidar_maze_navigation::msg::ExactPose>::SharedPtr
    exact_pose_pub;

enum class RobotState { LOCALIZING, TRACKING };
RobotState current_state = RobotState::LOCALIZING;

enum Wall { RIGHT = 0, TOP = 1, LEFT = 2, BOTTOM = 3 };

// normal distribution parameters for sensor model and motion model (am nevoie
// de toate???)
float expected_dist;
float expected_turning_angle;
float motion_go_stddev = 0.04;
float motion_turn_stddev = 0.10; // param-tuning: was 0.2, changed to 0.10
float dist_stddev = 0.12f;       // param tries: was 0.06, now 0.12

// FIX:
//  bool injection_enabled = false;   // only when in tracking not while
//  localizing

std::mt19937 gen(std::random_device{}());

int lost_counter = 0;
const int LOST_THRESHOLD = 100;

// Callbacks
void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
  latest_scan = msg;
  scan_received = true;
}

void ransac_callback(const lidar_maze_navigation::msg::Ransac &msg) {
  walls = msg.walls;
}

void map_callback(const lidar_maze_navigation::msg::Grid &msg) {
  map = msg.rows;
}

void sensor_callback(const create_fundamentals::msg::SensorPacket &msg) {
  real_encoder_left = msg.encoder_left;
  real_encoder_right = msg.encoder_right;
}

void motion_callback(const lidar_maze_navigation::msg::Motion &msg) {
  std::lock_guard<std::mutex> lock(motion_mutex);

  if (first_motion_msg) {
    last_seen_total_dist = msg.d_dist;
    last_seen_total_angle = msg.d_angle;
    first_motion_msg = false;
    return;
  }

  float delta_dist  = msg.d_dist  - last_seen_total_dist;
  float delta_angle = msg.d_angle - last_seen_total_angle;

  last_seen_total_dist  = msg.d_dist;
  last_seen_total_angle = msg.d_angle;
  
  pending_d_dist += delta_dist;
  pending_d_angle += delta_angle;
  motion_received = true;
}

// Our Encoders
void update_encoder() {
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt = now - last_time;
  last_time = now;
  encoder_left += current_cmd_left * dt.count();
  encoder_right += current_cmd_right * dt.count();
}

void reset_encoder() {
  // software encoders
  encoder_left = 0.0;
  encoder_right = 0.0;
  current_cmd_left = 0.0;
  current_cmd_right = 0.0;
  last_time = std::chrono::steady_clock::now();
  // echte encoder baseline merken
  encoder_left_baseline = real_encoder_left;
  encoder_right_baseline = real_encoder_right;
}

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

void play_success_sound() {
  std::system("ros2 service call /play_song create_fundamentals/srv/PlaySong "
              "\"{number: 2}\"");
}

// void cell_to_world_corner(int row, int col, float &x, float &y) {
//     x = col * cell_length;     // given a cell's grid index, returns the
//     world coordinates of its top left corner y = row * cell_length;
// }

// void cell_to_world_center(int row, int col, float &x, float &y){
//     x = col * cell_length + HALF_CELL;
//     y = row * cell_length + HALF_CELL;
// }

void world_to_cell(float x, float y, int &row, int &col) {
  row = (int)std::floor(y / cell_length);
  col = (int)std::floor(
      x / cell_length); // given a particle's continuous world coordinates,
                        // returns which grid cell it currently sits in
}

float calculate_speed(float dist, float dist_desired, float k) {
  return k * (dist_desired - dist);
}

float real_dist_traveled() {
  float enc_l = real_encoder_left - encoder_left_baseline;
  float enc_r = real_encoder_right - encoder_right_baseline;
  return r * (enc_l + enc_r) / 2.0f;
}

void send_drive(std::shared_ptr<rclcpp::Node> node, float left, float right) {
  auto request =
      std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
  auto diffDrive =
      node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive");
  current_cmd_left = left;
  current_cmd_right = right;
  while (!diffDrive->wait_for_service(std::chrono::milliseconds(200))) {
  }
  request->left = left;
  request->right = right;
  diffDrive->async_send_request(request);
}

void normalize_weights() {
  // normalize weights
  float total_weight = 0.0;
  for (Particle &p : particle_map) {
    total_weight += p.weight;
  }
  if (total_weight > 0.0f) {
    for (Particle &p : particle_map) {
      p.weight /= total_weight;
    }
  }
}

// specific grid: in each big cell, 10x10 mini cells
// on specific grid, save whether a mini cell is part of a wall or not
void initialize_wall_map() {
  int rows = map.size();
  int cols = map[0].cells.size();
  int scale = 10;

  int grid_rows = rows * scale;
  int grid_cols = cols * scale;
  wall_map.assign(
      grid_rows,
      std::vector<int>(grid_cols, 0)); // initialize all mini cells as free (0)

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      for (int wall : map[row].cells[col].walls) {
        if (wall == 0) { // RIGHT wall
          for (int i = 0; i < scale; i++)
            wall_map[row * scale + i][(col + 1) * scale - 1] = 1;
        } else if (wall == 1) { // TOP wall
          for (int i = 0; i < scale; i++)
            wall_map[row * scale][col * scale + i] = 1;
        } else if (wall == 2) { // LEFT wall
          for (int i = 0; i < scale; i++)
            wall_map[row * scale + i][col * scale] = 1;
        } else if (wall == 3) { // BOTTOM wall
          for (int i = 0; i < scale; i++)
            wall_map[(row + 1) * scale - 1][col * scale + i] = 1;
        }
      }
    }
  }
}

// export wall map to csv for visualization
void export_wall_map(const std::string &filename) {
  std::ofstream file(filename);
  for (const auto &row : wall_map) {
    for (int i = 0; i < row.size(); i++) {
      file << row[i];
      if (i < row.size() - 1)
        file << ",";
    }
    file << "\n";
  }
}

// initialize particles in non-wall mini-cells
// (TODO: if wall in mini-cell, don't put particle there)
void initialize_particles() {
  particle_map.clear();
  float orientations[4] = {0.0, 3.0 * M_PI_2, M_PI,
                           M_PI_2}; // 0- right 270 -up 180 - left - 90 -down
  float alignment_stddev = 10.0 * M_PI / 180.0;
  std::normal_distribution<double> angle_noise(0.0, alignment_stddev);
  std::uniform_real_distribution<float> pos_dist(
      0.0, cell_length); // we dont need normal_distribution at the start. wrong
  // we dont want most particle to be in a certain range, but equally
  // distributed. that's why -> uniform_real_distribution for distance and
  // normal_distribution for angle around the 4 orientations (0, PI/2, PI,
  // 3*PI/2)

  for (int row = 0; row < map.size(); row++) { // 3 times. one for each row
    for (int col = 0; col < map[0].cells.size();
         col++) { // each cell in the row
      float corner_x =
          col * cell_length; // starts the corner_coordinates as (0,0) and
                             // increments with incrementing row and cor
      float corner_y = row * cell_length;

      for (int k = 0; k < 100; k++) {
        Particle p;
        p.x = corner_x + pos_dist(gen);
        p.y = corner_y + pos_dist(gen);
        int attempts = 0;
        while (attempts++ < 20 && wall_map[(int)(p.y / mini_cell_length)][(
                                      int)(p.x / mini_cell_length)] == 1) {
          p.x = corner_x + pos_dist(gen);
          p.y = corner_y + pos_dist(gen);
        }
        p.angle = orientations[k % 4] + angle_noise(gen);
        p.weight = 0.0;
        particle_map.push_back(p);
      }
    }
  }

  // add initial weight based on uniform distribution
  for (Particle &p : particle_map) {
    p.weight = 1.0 / particle_map.size();
  }
}

void inject_random_particles(int n) {
  if (wall_map.empty())
    return;
  float orientations[4] = {0.0f, 3.0f * (float)M_PI_2, (float)M_PI,
                           (float)M_PI_2};
  std::normal_distribution<double> angle_noise(0.0, 15.0 * M_PI / 180.0);
  std::uniform_real_distribution<float> rx(0.0f, wall_map[0].size() *
                                                     mini_cell_length);
  std::uniform_real_distribution<float> ry(0.0f,
                                           wall_map.size() * mini_cell_length);
  std::uniform_int_distribution<int> ro(0, 3);

  int added = 0, attempts = 0;
  while (added < n && attempts < n * 20) {
    attempts++;
    Particle p;
    p.x = rx(gen);
    p.y = ry(gen);
    int mr = (int)(p.y / mini_cell_length);
    int mc = (int)(p.x / mini_cell_length);
    if (mr < 0 || mr >= (int)wall_map.size() || mc < 0 ||
        mc >= (int)wall_map[0].size())
      continue;
    if (wall_map[mr][mc] == 1)
      continue;
    p.angle = orientations[ro(gen)] + (float)angle_noise(gen);
    p.weight = 1.0f / (particle_map.size() + 1);
    particle_map.push_back(p);
    added++;
  }
}

void teleport_cluster(int row, int col, int orientation) {
  if (particle_map.empty()) return;
  float cx = col * cell_length + HALF_CELL;
  float cy = row * cell_length + HALF_CELL;
  float orient_angles[4] = {0.0f, 3.0f * (float)M_PI_2, (float)M_PI, (float)M_PI_2};
  float base_angle = orient_angles[orientation];

  std::normal_distribution<float> pos_noise(0.0f, 0.15f);          // ~4cm
  std::normal_distribution<float> ang_noise(0.0f, 40.0f * M_PI / 180.0f);

  for (Particle &p : particle_map) {
    p.x = cx + pos_noise(gen);
    p.y = cy + pos_noise(gen);
    p.angle = base_angle + ang_noise(gen);
    p.weight = 1.0f / particle_map.size();
  }
  prev_mean_x = cx;   // keep convergence tracker in sync so is_converged sees no fake jump
  prev_mean_y = cy;
  stable_count = 0;
}

void teleport_callback(const lidar_maze_navigation::msg::Pose &msg) {
  teleport_cluster(msg.row, msg.column, msg.orientation);
  RCLCPP_INFO(rclcpp::get_logger("localization"),
    "cloud teleported to row=%d col=%d orient=%d", msg.row, msg.column, msg.orientation);
}


// void cull_weak_particles(float threshold) {
//     std::vector<Particle> survivors;
//     survivors.reserve(particle_map.size());
//     for (const Particle &p : particle_map)
//         if (p.weight >= threshold) survivors.push_back(p);

//     if ((int)survivors.size() < 50) return;  // safety floor
//     particle_map = std::move(survivors);
//     normalize_weights();
// }

void export_particles(const std::string &filename) {
  std::ofstream file(filename);
  file << "x,y,angle,weight\n";
  for (const auto &p : particle_map) {
    file << p.x << "," << p.y << "," << p.angle << "," << p.weight << "\n";
  }
}

// FIX: signature changed from (Particle &p, float angle) to (float x, float y,
// float angle) to match the robot center's delta
float cast_ray(float x, float y, float angle) {
  float step = 0.01;
  float dist = 0.0;
  float max_range = 2.4;
  while (dist < max_range) {     // 2 cells max
    x += step * std::cos(angle); // step along the beam direction
    y += step * std::sin(angle);
    dist += step;
    int mini_row = (int)(y / mini_cell_length); // which mini-cell are we in
    int mini_col = (int)(x / mini_cell_length);
    if (mini_row < 0 || mini_row >= (int)wall_map.size() || mini_col < 0 ||
        mini_col >= (int)wall_map[0].size()) {
      return dist;
    }
    if (wall_map[mini_row][mini_col] == 1) {
      return dist; // hit a wall
    }
  }
  return max_range; // open corridor longer than 2 cells
}

// FIX: per-beam diagnostics for the current best (highest-weight) particle, for
// visualization
struct BeamDiag {
  float beam_angle; // angle in the sensor's own frame (latest_scan.angle_min +
                    // i*increment)
  float expected_dist; // ray-cast distance predicted by the particle's pose
  float actual_dist;   // measured distance from the scan
  float likelihood;    // L = Z_HIT * gauss + Z_RAND / MAX_RANGE for this beam
};
std::vector<BeamDiag> best_beam_diag;
float best_diag_x = 0.0f, best_diag_y = 0.0f, best_diag_angle = 0.0f;

void export_beam_diag(const std::string &filename) {
  std::ofstream file(filename);
  file << "px,py,pangle,beam_angle,expected_dist,actual_dist,likelihood\n";
  for (const auto &bd : best_beam_diag) {
    file << best_diag_x << "," << best_diag_y << "," << best_diag_angle << ","
         << bd.beam_angle << "," << bd.expected_dist << "," << bd.actual_dist
         << "," << bd.likelihood << "\n";
  }
}

// assign importance factors based on sensor model
void assign_weights() {

  // FIX: flooring the max punishment for bad particles
  const float Z_HIT = 0.85f;
  const float Z_RAND = 0.15f;
  const float MAX_RANGE = 2.4f;

  int beam_step = 15; // FIX: use every 15th beam, was 20
  float max_log_weight =
      -std::numeric_limits<float>::infinity(); // use log weights to avoid
                                               // weights becoming 0 during
                                               // multiplication

  float margin = 0.05; // 5cm tolerance
  float max_x = wall_map[0].size() * mini_cell_length;
  float max_y = wall_map.size() * mini_cell_length;

  for (Particle &p : particle_map) {
    // outside maze (beyond 5cm) -> kill it
    if (p.x < -margin || p.x > max_x + margin || p.y < -margin ||
        p.y > max_y + margin) {

      p.weight =
          -std::numeric_limits<float>::infinity(); // log space -> exp() = 0
      continue;
    }

    // FIX: cast from SENSOR position, not wheel axis
    float sx = p.x + delta * std::cos(p.angle); // try if flipped -> -delta
    float sy = p.y + delta * std::sin(p.angle);

    float log_weight = 0.0;
    int n_beams = 0;
    std::vector<BeamDiag>
        cur_beams; // FIX: per-beam diagnostics for visualization
    for (int i = 0; i < latest_scan.ranges.size(); i += beam_step) {
      float beam_angle =
          latest_scan.angle_min + latest_scan.angle_increment * i;
      float world_angle = p.angle - beam_angle;

      // FIX: cast_ray now takes sensor position sx, sy instead of particle p
      float expected_dist =
          cast_ray(sx, sy, world_angle); // expected distance measurement for
                                         // this particle and beam
      float actual_dist = latest_scan.ranges[i];

      if (!std::isfinite(actual_dist)) {
        continue;
      }

      // FIX: floors max punishment for bad particles
      float diff = actual_dist - expected_dist;
      float gauss = std::exp(-0.5f * diff * diff / (dist_stddev * dist_stddev));
      float L = Z_HIT * gauss + Z_RAND / MAX_RANGE;
      log_weight += std::log(L);

      cur_beams.push_back({beam_angle, expected_dist, actual_dist,
                           L}); // FIX: stash for diagnostics

      // n_beams++;
    }

    // if (n_beams > 0) log_weight /= n_beams;  <-- average, not sum
    p.weight = log_weight; // sum

    if (log_weight > max_log_weight) {
      max_log_weight = log_weight;

      best_beam_diag = cur_beams;

      best_diag_x = p.x;
      best_diag_y = p.y;
      best_diag_angle = p.angle;
    }
  }

  // turn weights back from log space and normalize
  for (Particle &p : particle_map) {
    p.weight = std::exp(p.weight -
                        max_log_weight); // subtract max for numerical stability
  }

  normalize_weights();
}

// re-sample to adapt distribution of samples according to importance factors
const int TARGET_PARTICLES =
    900; // set to your initial count (100 * number of cells)

// FIX: added \u2014 5% fresh random particles per resample
const float INJECT_FRAC = 0.4f;

// void resample() {
//     int N = particle_map.size();
//     if (N == 0) return;

//     float total = 0.0f;
//     for (const Particle &p : particle_map) total += p.weight;
//     if (total <= 0.0f) return;

//     int M = TARGET_PARTICLES;                 // always produce a fixed count
//     std::vector<Particle> new_particles;
//     new_particles.reserve(M);

//     float step = total / M;
//     float r = std::uniform_real_distribution<float>(0.0f, step)(gen);
//     float c = particle_map[0].weight;
//     int i = 0;
//     for (int m = 0; m < M; m++) {
//         float U = r + m * step;               // evenly spaced, single random
//         offset while (U > c && i < N - 1) {
//             i++;
//             c += particle_map[i].weight;
//         }
//         new_particles.push_back(particle_map[i]);
//     }
//     particle_map = std::move(new_particles);
//     normalize_weights();
// }

void resample() {
  int N = particle_map.size();
  if (N == 0)
    return;

  float total = 0.0f;
  for (const Particle &p : particle_map)
    total += p.weight;
  if (total <= 0.0f)
    return;
  int M_inject = 0;
  int M = TARGET_PARTICLES;
  // FIX: injection_enabled is false during initial localization. set to true
  // after publish_pose() in run_localization int M_inject = injection_enabled ?
  // (int)(INJECT_FRAC * M) : 0;   // 45 fresh random particles
  int M_sus = M;
  if(current_state == RobotState::LOCALIZING){
    M_inject = (int)(INJECT_FRAC * M); // 360 fresh random particles  
  }
  M_sus = M - M_inject;   
  std::vector<Particle> new_particles;
  new_particles.reserve(M);

  // SUS loop. unchanged logic, just runs M_sus times instead of M
  float step = total / M_sus;
  float r = std::uniform_real_distribution<float>(0.0f, step)(gen);
  float c = particle_map[0].weight;
  int i = 0;
  for (int m = 0; m < M_sus; m++) {
    float U = r + m * step;
    while (U > c && i < N - 1) {
      i++;
      c += particle_map[i].weight;
    }
    new_particles.push_back(particle_map[i]);
  }

  // FIX: inject M_inject uniform-random free-space particles for kidnap
  // recovery
  float orientations[4] = {0.0f, 3.0f * M_PI_2, M_PI, M_PI_2};
  std::uniform_real_distribution<float> rx(0.0f, wall_map[0].size() *
                                                     mini_cell_length);
  std::uniform_real_distribution<float> ry(0.0f,
                                           wall_map.size() * mini_cell_length);
  std::uniform_int_distribution<int> ro(0, 3);
  std::normal_distribution<float> rangle(0.0f, 15.0f * M_PI / 180.0f);

  int added = 0, attempts = 0;
  while (added < M_inject && attempts < M_inject * 20) {
    attempts++;
    Particle p;
    p.x = rx(gen);
    p.y = ry(gen);
    int mr = (int)(p.y / mini_cell_length);
    int mc = (int)(p.x / mini_cell_length);
    if (mr < 0 || mr >= (int)wall_map.size() || mc < 0 ||
        mc >= (int)wall_map[0].size())
      continue;
    if (wall_map[mr][mc] == 1)
      continue;
    p.angle = orientations[ro(gen)] + rangle(gen);
    p.weight = 1.0f / M; // overwritten by next assign_weights anyway
    new_particles.push_back(p);
    added++;
  }

  particle_map = std::move(new_particles);
  normalize_weights();
}

void roughen(float pos_sigma, float ang_sigma) {
  std::normal_distribution<float> np(0.0f, pos_sigma);
  std::normal_distribution<float> na(0.0f, ang_sigma);
  for (Particle &p : particle_map) {
    p.x += np(gen);
    p.y += np(gen);
    p.angle += na(gen);
    while (p.angle >= 2 * M_PI)
      p.angle -= 2 * M_PI;
    while (p.angle < 0)
      p.angle += 2 * M_PI;
  }
}

float effective_sample_size() {
  float total = 0.0f;
  for (const Particle &p : particle_map)
    total += p.weight;
  if (total <= 0.0f)
    return 0.0f;
  float sum_sq = 0.0f;
  for (const Particle &p : particle_map) {
    float w = p.weight / total;
    sum_sq += w * w;
  }
  return 1.0f / sum_sq; // ranges 1 (collapsed) .. N (uniform)
}

// predict samples after action based on motion model
void move_particles(float d_dist, float d_angle) {
  float go_std = motion_go_stddev * std::fabs(d_dist) + 0.005f;
  float turn_std = motion_turn_stddev * std::fabs(d_angle) + 0.01f;
  std::normal_distribution<double> mg(d_dist, go_std);
  std::normal_distribution<double> mt(d_angle, turn_std);
  for (Particle &p : particle_map) {
    if (d_angle != 0.0f) {
      p.angle += mt(gen);
      while (p.angle >= 2 * M_PI)
        p.angle -= 2 * M_PI;
      while (p.angle < 0)
        p.angle += 2 * M_PI;
    }
    if (d_dist != 0.0f) {
      float dist = mg(gen);
      p.x += dist * std::cos(p.angle);
      p.y += dist * std::sin(p.angle);
    }
  }
}

bool is_converged(float var_threshold, bool use_jump = true) {
  float mean_x = 0, mean_y = 0;
  for (auto &p : particle_map) {
    mean_x += p.x;
    mean_y += p.y;
  }
  mean_x /= particle_map.size();
  mean_y /= particle_map.size();

  float variance = 0;
  for (auto &p : particle_map) {
    float dx = p.x - mean_x;
    float dy = p.y - mean_y;
    variance += dx * dx + dy * dy;
  }
  variance /= particle_map.size();
  float jump = std::hypot(mean_x - prev_mean_x, mean_y - prev_mean_y);
  prev_mean_x = mean_x;
  prev_mean_y = mean_y;

  bool condition = variance < var_threshold;
  if (use_jump)
    condition = condition && jump < 0.75f;

  if (condition)
    stable_count++; // param-tuning: variance was 0.08, now 0.1
  else
    stable_count = 0;

  RCLCPP_INFO(rclcpp::get_logger("localization"),
              "mean=(%.2f,%.2f) var=%.3f jump=%.2f stable=%d", mean_x, mean_y,
              variance, jump, stable_count);

  return stable_count >= 10; // 3 consecutive low-variance, low-jump scans
}

void get_pose(float &x, float &y, float &angle) {
  int best_count = 0;
  int best_idx = 0;
  for (int i = 0; i < (int)particle_map.size(); i++) {
    int count = 0;
    for (int j = 0; j < (int)particle_map.size(); j++) {
      float dx = particle_map[j].x - particle_map[i].x;
      float dy = particle_map[j].y - particle_map[i].y;
      float da = std::fabs(particle_map[j].angle - particle_map[i].angle);
      while (da > M_PI) {
        da = abs(da - 2 * M_PI);
      }
      if (std::sqrt(dx * dx + dy * dy) < 0.20f && da < 0.3f)
        count++;
    }
    if (count > best_count) {
      best_count = count;
      best_idx = i;
    }
  }
  x = particle_map[best_idx].x;
  y = particle_map[best_idx].y;
  angle = particle_map[best_idx].angle;
}

void publish_pose(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::Publisher<lidar_maze_navigation::msg::Pose>::SharedPtr pose_pub) {
  float bx, by, ba;
  get_pose(bx, by, ba);
  int mrow = (int)std::floor(by / mini_cell_length);
  int mcol = (int)std::floor(bx / mini_cell_length);
  lidar_maze_navigation::msg::ExactPose exact_pose;
  exact_pose.current_row = mrow;
  exact_pose.current_col = mcol;
  exact_pose_pub->publish(exact_pose);
  int brow, bcol;
  world_to_cell(bx, by, brow, bcol);

  float angles[4] = {0.0, 3.0 * M_PI_2, M_PI, M_PI_2};
  int orientation = 0;
  float best_diff = 1e9;
  for (int i = 0; i < 4; i++) {
    float diff = std::fabs(ba - angles[i]);
    if (diff > M_PI)
      diff = 2 * M_PI - diff;
    if (diff < best_diff) {
      best_diff = diff;
      orientation = i;
    }
  }

  lidar_maze_navigation::msg::Pose m;
  m.row = brow;
  m.column = bcol;
  m.orientation = orientation;
  RCLCPP_INFO(node->get_logger(),
              "Pose: row=%d col=%d orientation=%d | mini: row=%d col=%d", brow,
              bcol, orientation, mrow, mcol);
  pose_pub->publish(m);
}

float threshold = 0.40;
bool wall_in_front(std::shared_ptr<rclcpp::Node> node) {
  if (!scan_received)
    return false;
  int total = latest_scan.ranges.size();
  int front_half = (int)(10.0 / (240.0 / total)); // 20 degree scan
  int front_start = total / 2 - front_half;
  int front_end = total / 2 + front_half;

  for (int i = front_start; i <= front_end; i++) {
    float r = latest_scan.ranges[i];
    if (std::isfinite(r) && r < threshold && r > 0.10) {
      RCLCPP_INFO(node->get_logger(), "wall in forn: %f", r);
      return true;
    }
  }
  return false;
}

struct Surroundings {
  float front, front_left, front_right, left, right;
};

Surroundings sense() {
  Surroundings s{1e9f, 1e9f, 1e9f, 1e9f, 1e9f};
  int total = latest_scan.ranges.size();
  float step = 240.0f / total;

  int fh = (int)(10.0 / step);
  int fs = total / 2 - fh, fe = total / 2 + fh;
  int fls = fs - (int)(25.0 / step), fle = fs;
  int frs = fe, fre = fe + (int)(25.0 / step);
  int rs = total / 2 - (int)(100.0 / step);
  int re = total / 2 - (int)(30.0 / step);
  int ls = total / 2 + (int)(30.0 / step);
  int le = total / 2 + (int)(100.0 / step);

  auto scan = [&](int a, int b, float &out) {
    for (int i = a; i <= b && i < total; i++) {
      float v = latest_scan.ranges[i];
      if (std::isfinite(v) && v > 0.07f && v < out)
        out = v;
    }
  };
  scan(fs, fe, s.front);
  scan(fls, fle, s.front_left);
  scan(frs, fre, s.front_right);
  scan(rs, re, s.right);
  scan(ls, le, s.left);
  return s;
}

const float V_LIN = 5.4f * r;               // ~0.173 m/s
const float OMEGA = (2.0f * 3.18f * r) / b; // rad/s for \u00b13.18 wheels

float step_forward(std::shared_ptr<rclcpp::Node> node) {
  float start_dist = real_dist_traveled();
  float last_update_dist = start_dist;          // for incremental motion
  send_drive(node, 7.5f, 7.5f);

  while (rclcpp::ok()) {
    executor->spin_some();

    Surroundings s = sense();
    if (s.front <= 0.30f || s.front_left <= 0.22f || s.front_right <= 0.22f)
      break;

    // --- mid-drive filter update ---
    float now_dist = real_dist_traveled();
    float dinc = now_dist - last_update_dist;
    if (dinc >= 0.2f) {                          // every ~5 cm of travel
      move_particles(dinc, 0.0f);                 // propagate the increment
      last_update_dist = now_dist;

      // need a fresh scan to weight against
      if (scan_received) {
        scan_received = false;
        assign_weights();
        bool conv = is_converged(3.5,true);              // updates jump/variance/stable + logs
        if (effective_sample_size() < 0.5f * particle_map.size()) {
          resample();
          if (!conv) roughen(0.02f, 0.05f);
        }
      }
    }

    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  send_drive(node, 0.0f, 0.0f);

  // return only the distance NOT yet propagated, so the caller's
  // move_particles(d,0) doesn't double-count what we already moved
  float total = real_dist_traveled() - start_dist;
  float already = last_update_dist - start_dist;
  return total - already;
}

// turn one short step toward freer side. returns signed radians
// turn one short step toward freer side. returns signed radians
float step_turn(std::shared_ptr<rclcpp::Node> node) {
  reset_encoder();
  Surroundings s = sense();

  bool ccw = (s.left > s.right);
  float left_margin = s.left - s.right;
  bool ambiguous = std::fabs(left_margin) < 0.05f;

  // escalate the turn size the longer we've been stuck
  float target;
  if (consecutive_turns >= 4) {
    // truly trapped: commit to a big turn toward the freer side to break out
    target = M_PI;            // ~180 deg
    ccw = (s.left > s.right); // freest side, full commit (don't oscillate)
  } else if (consecutive_turns >= 2) {
    target = M_PI_2;
    ccw = ambiguous ? (std::uniform_int_distribution<int>(0, 1)(gen) == 0)
                    : (s.left > s.right);
  } else {
    target = M_PI_2;
    ccw = ambiguous ? (std::uniform_int_distribution<int>(0, 1)(gen) == 0)
                    : (s.left > s.right);
  }

  if (ccw)
    send_drive(node, -3.18f, 3.18f);
  else
    send_drive(node, 3.18f, -3.18f);

  while (rclcpp::ok()) {
    executor->spin_some();
    float enc_l = real_encoder_left - encoder_left_baseline;
    float enc_r = real_encoder_right - encoder_right_baseline;
    float angle = std::fabs(r * (enc_r - enc_l) / b);
    if (angle >= target)
      break;
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  send_drive(node, 0.0f, 0.0f);

  last_turn_dir = ccw ? -1 : +1;
  consecutive_turns++;
  if (consecutive_turns > 8)
    consecutive_turns = 0; // restart escalation, don't freeze

  float enc_l = real_encoder_left - encoder_left_baseline;
  float enc_r = real_encoder_right - encoder_right_baseline;
  float turned = r * (enc_r - enc_l) / b; // signed
  return ccw ? -std::fabs(turned) : std::fabs(turned);
}

void align_to_map(std::shared_ptr<rclcpp::Node> node) {
  float bx, by, ba;
  get_pose(bx, by, ba);

  float target_angle = 3 * M_PI_2; // orientation 1 = up

  float diff = target_angle - ba;
  while (diff > M_PI)
    diff -= 2 * M_PI;
  while (diff < -M_PI)
    diff += 2 * M_PI;

  if (std::fabs(diff) < 0.05f)
    return;

  reset_encoder();

  if (diff > 0)
    send_drive(node, 3.18f, -3.18f);
  else
    send_drive(node, -3.18f, 3.18f);

  while (rclcpp::ok()) {
    executor->spin_some();
    float enc_l = real_encoder_left - encoder_left_baseline;
    float enc_r = real_encoder_right - encoder_right_baseline;
    float turned = std::fabs(r * (enc_r - enc_l) / b);
    if (turned >= std::fabs(diff))
      break;
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  send_drive(node, 0.0f, 0.0f);
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  float enc_l = real_encoder_left - encoder_left_baseline;
  float enc_r = real_encoder_right - encoder_right_baseline;
  float actual_turned = -r * (enc_r - enc_l) / b;
  move_particles(0.0f, actual_turned);
}

// clear old data from plotter
void clear_csv_files() {
  std::vector<std::string> files = {
      "src/lidar_maze_navigation/particles.csv",
      "src/lidar_maze_navigation/wall_map.csv",
      "src/lidar_maze_navigation/pot_field.csv",
      "src/lidar_maze_navigation/path.csv",
      "src/lidar_maze_navigation/subgoals.csv",
      "src/lidar_maze_navigation/gold_locations.csv",
      "src/lidar_maze_navigation/goal.csv",
      "src/lidar_maze_navigation/attractive.csv",
  };
  for (const auto &f : files) {
    std::ofstream(f, std::ofstream::trunc).close(); // truncate to empty
  }
}

bool has_wall(const std::vector<int> &walls, int w) {
  return std::find(walls.begin(), walls.end(), w) != walls.end();
}

void compute_open_cells() {
  int num_rows = map.size();
  int num_cols = map[0].cells.size();
  open_cell.assign(num_rows, std::vector<bool>(num_cols, false));
  for (int r = 0; r + 1 < num_rows; r++) {
    for (int c = 0; c + 1 < num_cols; c++) {
      const auto &TL = map[r].cells[c].walls;     // top-left
      const auto &TR = map[r].cells[c + 1].walls; // top-right
      const auto &BL = map[r + 1].cells[c].walls; // bottom-left
      // interior walls of the 2x2 must all be ABSENT:
      bool open = !has_wall(TL, RIGHT) &&  // TL|TR vertical seam
                  !has_wall(TL, BOTTOM) && // TL/BL horizontal seam
                  !has_wall(TR, BOTTOM) && // TR/BR horizontal seam
                  !has_wall(BL, RIGHT);    // BL|BR vertical seam
      if (open) {
        open_cell[r][c] = true;
        open_cell[r][c + 1] = true;
        open_cell[r + 1][c] = true;
        open_cell[r + 1][c + 1] = true;
      }
    }
  }
}

float dist_since_weight = 0.0f;

void run_localization(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::Publisher<lidar_maze_navigation::msg::Pose>::SharedPtr pose_pub) {
  current_state = RobotState::LOCALIZING;
  initialize_particles();
  prev_mean_x = 1e9f; // reset convergence tracker
  prev_mean_y = 1e9f;
  stable_count = 0;

  rclcpp::sleep_for(std::chrono::seconds(4));
  float total_dist = 0.0f;

  float total_angle = 0.0f;
  const float MIN_DIST = 0.60f; // tune: meters moved before checking
  const float MIN_ANGLE = 1.6f; // tune: radians turned before checking
  export_particles( "src/lidar_maze_navigation/particles.csv");

  bool moved_enough = false;
  int settle_iters = 1;

  while (rclcpp::ok()) {
    for (int k = 0; k < settle_iters; k++) {
      executor->spin_some();
      assign_weights();
      export_particles("src/lidar_maze_navigation/particles.csv");
      export_beam_diag("src/lidar_maze_navigation/beam_diag.csv");

      bool conv = is_converged(3.5, true);
      if (effective_sample_size() < 0.5f * particle_map.size()) {
        resample();
        if (!conv) roughen(0.02f, 0.05f);
        export_particles("src/lidar_maze_navigation/particles.csv");
      }
      bool moved_enough   = (total_dist >= MIN_DIST);
      bool stuck_but_sure = (stable_count >= 6);
      if ((moved_enough || stuck_but_sure) && conv) {
        play_success_sound();
        goto done;
      }
    }

    Surroundings s = sense();
    bool blocked = (s.front <= 0.30f || s.front_left <= 0.22f || s.front_right <= 0.22f);
    float d = 0, a = 0;

    if (!blocked) {
      d = step_forward(node);        // weights internally; returns unpropagated remainder
      move_particles(d, 0.0f);       // finish the last sub-5cm
      total_dist += d;
      consecutive_turns = 0;
    } else {
      a = step_turn(node);
      move_particles(0.0f, a);
      total_angle += std::fabs(a);
    }

    // (HALF_CELL block deleted \u2014 step_forward now does mid-drive weighting)

    export_particles("src/lidar_maze_navigation/particles.csv");

    scan_received = false;
    while (!scan_received && rclcpp::ok()) {
      executor->spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }
done: {
  std::lock_guard<std::mutex> lock(motion_mutex);
  pending_d_dist = 0.0f;
  pending_d_angle = 0.0f;
  motion_received = false;
}

  // publish pose
  publish_pose(node, pose_pub);
  current_state = RobotState::TRACKING;
  // injection_enabled = true;  // FIX: turn injection ON now that localization
  // is done
  RCLCPP_INFO(node->get_logger(), "Initial localization done.");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("localization");

  auto sensor_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // auto ransac_cbg =
  // node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto map_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sensor_packet_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto motion_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto teleport_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto reset_cbg =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sensor_opts;
  sensor_opts.callback_group = sensor_cbg;
  auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", 10,
      [&](const sensor_msgs::msg::LaserScan &msg) { laser_callback(msg); },
      sensor_opts);

  // rclcpp::SubscriptionOptions ransac_opts;
  // ransac_opts.callback_group = ransac_cbg;
  // auto ransac_sub =
  // node->create_subscription<lidar_maze_navigation::msg::Ransac>(
  //     "ransac", 10,
  //     [&](const lidar_maze_navigation::msg::Ransac &msg) {
  //     ransac_callback(msg); }, ransac_opts);

  rclcpp::SubscriptionOptions map_opts;
  map_opts.callback_group = map_cbg;
  auto map_sub = node->create_subscription<lidar_maze_navigation::msg::Grid>(
      "map", 10,
      [&](const lidar_maze_navigation::msg::Grid &msg) { map_callback(msg); },
      map_opts);

  rclcpp::SubscriptionOptions sensor_packet_opts;
  sensor_packet_opts.callback_group = sensor_packet_cbg;
  auto sensor_packet_sub =
      node->create_subscription<create_fundamentals::msg::SensorPacket>(
          "sensor_packet", 10,
          [&](const create_fundamentals::msg::SensorPacket &msg) {
            sensor_callback(msg);
          },
          sensor_packet_opts);

  rclcpp::SubscriptionOptions motion_opts;
  motion_opts.callback_group = motion_cbg;
  auto motion_sub =
      node->create_subscription<lidar_maze_navigation::msg::Motion>(
          "motion_delta", 600,
          [&](const lidar_maze_navigation::msg::Motion &msg) {
            motion_callback(msg);
          },
          motion_opts);

  rclcpp::SubscriptionOptions teleport_opts;
  teleport_opts.callback_group = teleport_cbg;
  auto teleport_sub = 
      node->create_subscription<lidar_maze_navigation::msg::Pose>(
        "teleport_to", 10,
        [&](const lidar_maze_navigation::msg::Pose &msg) { teleport_callback(msg); },
        teleport_opts);

  auto lost_pub = node->create_publisher<std_msgs::msg::Bool>(
      "localization_status_lost", 10);
  auto pose_pub =
      node->create_publisher<lidar_maze_navigation::msg::Pose>("pose", 10);
  exact_pose_pub =
      node->create_publisher<lidar_maze_navigation::msg::ExactPose>(
          "exact_pose", 10);
  executor->add_node(node);

  clear_csv_files();
  initialize_sounds();

  std::atomic<bool> relocalize_requested{false};
  auto reset_service = node->create_service<std_srvs::srv::Trigger>(
      "reset_global_localization",
      [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        // run_localization(node, pose_pub);     do NOT run localization here
        // align_to_map(node);
        relocalize_requested = true;
        res->success = true;
      },
      rmw_qos_profile_services_default, reset_cbg);

  // wait for map
  while (map.empty() && rclcpp::ok()) {
    executor->spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  initialize_wall_map();
  compute_open_cells();
  export_wall_map(
      "src/lidar_maze_navigation/wall_map.csv");
  // wait for first scan
  while (!scan_received && rclcpp::ok()) {
    executor->spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  run_localization(node, pose_pub);
  align_to_map(node);

  follow_plan_client =
      node->create_client<std_srvs::srv::Trigger>("follow_plan");
  if (follow_plan_client->wait_for_service(std::chrono::seconds(2))) {
    follow_plan_client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
  }
    

  // keep spinning so subscribers stay alive
  rclcpp::Rate track_rate(20);
  while (rclcpp::ok()) {

    if (relocalize_requested) {
      relocalize_requested = false;
      RCLCPP_INFO(node->get_logger(),
                  "==========================================================");
      RCLCPP_INFO(node->get_logger(), "Starting Global Relocalization...");
      RCLCPP_INFO(node->get_logger(),
                  "==========================================================");
      run_localization(node, pose_pub);
      align_to_map(node);

      follow_plan_client = node->create_client<std_srvs::srv::Trigger>("follow_plan");
      if (follow_plan_client->wait_for_service(std::chrono::seconds(2))) {
        follow_plan_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }

      std_msgs::msg::Bool lost_msg;
      lost_msg.data = false;
      lost_pub->publish(lost_msg);

      relocalize_requested = false;

      RCLCPP_INFO(node->get_logger(),
                  "Relocalization complete. Resuming tracking.");
    }

    executor->spin_some();

    // apply motion immediately whenever it arrives
    if (motion_received) {
      float d, a;
      {
        std::lock_guard<std::mutex> lock(motion_mutex);
        d = pending_d_dist;
        a = pending_d_angle;
        pending_d_dist = 0.0f;
        pending_d_angle = 0.0f;
        motion_received = false;
      }
      RCLCPP_INFO(rclcpp::get_logger("dist from tracking:"), "d=(%.2f) a=%.3f",
                  d, a);
      move_particles(d, -a);
    }

    // laser update independently whenever a scan arrives
    if (scan_received) {
      scan_received = false;

      //  odometry-only in open cells
      float ex, ey, ea;
      get_pose(ex, ey, ea); // current estimate
      int erow, ecol;
      world_to_cell(ex, ey, erow, ecol);
      bool in_open = (!open_cell.empty() && erow >= 0 &&
                      erow < (int)open_cell.size() && ecol >= 0 &&
                      ecol < (int)open_cell[0].size() && open_cell[erow][ecol]);

      if (!in_open) {
        assign_weights();
        if (effective_sample_size() < 0.5f * particle_map.size()) {
          resample(); // only when genuinely concentrated
          roughen(0.03f, 0.08f);
        }
      }

      export_particles("src/"
                       "lidar_maze_navigation/particles.csv");
      publish_pose(node, pose_pub);

      if (current_state == RobotState::TRACKING) {
        bool high_variance = !is_converged(5.0, false);

        if (high_variance) {
          lost_counter++;
        } else {
          lost_counter = 0; // Reset if the filter converges again
        }
        bool lost = (lost_counter >= LOST_THRESHOLD);

        std_msgs::msg::Bool lost_msg;
        lost_msg.data = lost;
        lost_pub->publish(lost_msg);
        if (lost) {
          RCLCPP_INFO(node->get_logger(), "==========================================================");
          RCLCPP_INFO(node->get_logger(), "                          LOST");
          RCLCPP_INFO(node->get_logger(), "==========================================================");
        }
      }
    }
    track_rate.sleep();
  }
  rclcpp::shutdown();
}
