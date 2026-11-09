#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "lidar_maze_navigation/srv/execute_plan.hpp"  
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar_maze_navigation/msg/ransac.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <chrono>

// ================================================================================
// GLOBAL VARIABLES
// ================================================================================
const float r = 0.032;
const float b = 0.258;
const float delta = 0.128;
const float CELL_SIZE = 0.80;
const float HALF_CELL = CELL_SIZE / 2.0;

float encoder_left = 0.0;
float encoder_right = 0.0;
float current_cmd_left = 0.0;
float current_cmd_right = 0.0;
std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

sensor_msgs::msg::LaserScan latest_scan;
bool scan_received = false;

std::vector<lidar_maze_navigation::msg::Line> walls;

rclcpp::Rate rate(100);

// current heading state
int state = 1;

// ================================================================================
// LASER
// ================================================================================
void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
  latest_scan = msg;
  scan_received = true;
}
// RANSAC Callback

void ransac_callback(const lidar_maze_navigation::msg::Ransac &msg) {
    walls = msg.walls;
}

// ================================================================================
// ENCODERS
// ================================================================================
void update_encoder() {
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt = now - last_time;
  last_time = now;
  encoder_left += current_cmd_left * dt.count();
  encoder_right += current_cmd_right * dt.count();
}

void reset_encoder() {
    encoder_left = 0.0;
    encoder_right = 0.0;
    last_time = std::chrono::steady_clock::now();
}

// ================================================================================
// HELPERS FOR WALL DETECTION & RANSAC
// ================================================================================
struct WallInfo {
    bool found;
    float distance;
    float angle;
};

// RANSAC algorithm to fit a line to local laser points
WallInfo ransac(const std::vector<std::pair<float, float>>& points) {
    WallInfo result = {false, -1.0f, 0.0f};
    if (points.size() < 10) return result;

    int max_inliers = 0;
    float best_dist = 0.0f;
    float best_angle = 0.0f;
    
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, points.size() - 1);

    for (int i = 0; i < 50; i++) {
        int p1_idx = dist(rng);
        int p2_idx = dist(rng);
        if (p1_idx == p2_idx) continue;

        float x1 = points[p1_idx].first;
        float y1 = points[p1_idx].second;
        float x2 = points[p2_idx].first;
        float y2 = points[p2_idx].second;
        
        float dx = x2 - x1;
        float dy = y2 - y1;
        float len = std::hypot(dx, dy);
        if (len < 0.01f) continue;
        
        // Line equation: a*x + b*y - c = 0
        float a = -dy / len;
        float b = dx / len;
        float c = a * x1 + b * y1;

        int inliers = 0;
        for (const auto& p : points) {
            float d = std::abs(a * p.first + b * p.second - c);
            if (d < 0.03f) { // 3cm distance threshold for inliers
                inliers++;
            }
        }

        if (inliers > max_inliers) {
            max_inliers = inliers;
            best_dist = std::abs(c);
            
            // Normalize angle to [-pi/2, pi/2] so it represents the wall slope
            // float line_angle = std::atan2(dy, dx);
            float line_angle = std::atan2(b, a);
            // if (line_angle > M_PI_2) line_angle -= M_PI;
            // if (line_angle < -M_PI_2) line_angle += M_PI;
            best_angle = line_angle;
        }
    }

    if (max_inliers > points.size() * 0.5) { // Require at least 50% inliers
        result.found = true;
        result.distance = best_dist;
        result.angle = best_angle;
        // NORMALIZE
        if (result.angle < 0) {result.angle += M_PI;}
    }
    return result;
}

float front_wall_distance(float tol_deg = 20.0f) {
    float tol = tol_deg * M_PI / 180.0f;
    float min_dist = -1.0f;
    for (const auto &w : walls) {
        float diff_zero = std::fabs(w.angle);
        float diff_2pi  = std::fabs(w.angle - 2.0f * M_PI);
        if (diff_zero < tol || diff_2pi < tol) {
            if (min_dist < 0.0f || w.distance < min_dist) {
                min_dist = w.distance;
            }
        }
    }
    return min_dist;
}

WallInfo check_wall(std::shared_ptr<rclcpp::Node> node, float center_degree, float threshold = 0.6f, bool is_front = false) {
    WallInfo result = {false, -1.0f, 0.0f};

    if (!scan_received) return result;
    
    int total = latest_scan.ranges.size();
    if (total == 0) return result;

    float lookup_degree = 40.0f;
    float lookup_index_half = total * (lookup_degree / 2.0f / 240.0f);

    int scan_center = (int)(total * (center_degree / 240.0f)); 
    int scan_start = std::max(0, scan_center - (int)lookup_index_half);
    int scan_end = std::min(total - 1, scan_center + (int)lookup_index_half);

    std::vector<std::pair<float, float>> points;

    for (int i = scan_start; i <= scan_end; i++) {
        float r_val = latest_scan.ranges[i];
        
        if (std::isfinite(r_val) && r_val > latest_scan.range_min && r_val < latest_scan.range_max) {
            if (r_val < threshold) {
                // Map the index to the 240-degree sweep: [-30 deg, 210 deg]
                float theta_deg = -30.0f + ((float)i / (total - 1)) * 240.0f;
                float theta_rad = theta_deg * M_PI / 180.0f;
                
                // Extract general Cartesian coordinates
                float x = r_val * std::cos(theta_rad);
                float y = r_val * std::sin(theta_rad);
                points.push_back({x, y});
            }
        }
    }

    // for (const auto& point : points) {
    //     RCLCPP_INFO(node->get_logger(), "x: %f, y: %f", point.first, point.second);
    // }


    WallInfo ransac_res = ransac(points);

    if (ransac_res.found) {
        result.found = true;
        result.distance = ransac_res.distance;
        
        // Calculate the heading error based on wall position
        if (is_front) {
            result.angle = ransac_res.angle - M_PI_2;
            result.angle = ransac_res.angle;
            // RCLCPP_INFO(node->get_logger(), "check_wall [Front] angle: %f, %f / dist: %f m", ransac_res.angle * 180 / M_PI, result.angle * 180 / M_PI, result.distance);
        } else {
            if (ransac_res.angle > M_PI_2) {
                result.angle = ransac_res.angle - M_PI;
                result.angle = ransac_res.angle;
                // RCLCPP_INFO(node->get_logger(), "check_wall [Side] angle: %f, %f / dist: %f m", ransac_res.angle * 180 / M_PI, result.angle * 180 / M_PI, result.distance);
            } else {
                result.angle = ransac_res.angle;
                // RCLCPP_INFO(node->get_logger(), "check_wall [Side] angle: %f, %f / dist: %f m", ransac_res.angle * 180 / M_PI, result.angle * 180 / M_PI, result.distance);
            }
        }
    }

    return result;
}

// ================================================================================
// HELPERS FOR MOVING PATH
// ================================================================================

void send_drive(std::shared_ptr<rclcpp::Node> node, float left, float right) {
    auto request = std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
    auto diffDrive = node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive");

    current_cmd_left = left;
    current_cmd_right = right;

    while(!diffDrive->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_INFO(node->get_logger(), "waiting for service..");
    }
    
    request->left = left;
    request->right = right;
    diffDrive->async_send_request(request);
}

float calculate_speed(float dist, float dist_desired, float k) {
    return k * (dist_desired - dist);
}

// Aligns the robot's heading in place if the error is too large
void align_heading(std::shared_ptr<rclcpp::Node> node) {
    RCLCPP_INFO(node->get_logger(), "\n\nChecking heading alignment...");
    reset_encoder();
    while (rclcpp::ok()) {
        WallInfo front = check_wall(node, 120.0f, 0.60f, true);
        if (front.found) {RCLCPP_INFO(node->get_logger(), "[Front] angle: %f / dist: %f m", front.angle * 180 / M_PI, front.distance);}
        else {RCLCPP_INFO(node->get_logger(), "[Front] -");}

        WallInfo left  = check_wall(node, 210.0f, 0.60f, false);
        if (left.found) {RCLCPP_INFO(node->get_logger(), "[Left] angle: %f / dist: %f m", left.angle * 180 / M_PI, left.distance);}
        else {RCLCPP_INFO(node->get_logger(), "[Left] -");}

        WallInfo right = check_wall(node, 30.0f, 0.60f, false);
        if (right.found) {RCLCPP_INFO(node->get_logger(), "[Right] angle: %f / dist: %f m", right.angle * 180 / M_PI, right.distance);}
        else {RCLCPP_INFO(node->get_logger(), "[Right] -");}

        float error = 0.0f;
        bool can_align = false;

        // Prioritize walls that give a good line fit
        if (front.found) { 
            RCLCPP_INFO(node->get_logger(), "Aligning with respect to front wall.");
            error = front.angle; can_align = true; 
        } else if (left.found && right.found) { 
            RCLCPP_INFO(node->get_logger(), "Aligning with respect to left and right wall.");
            error = (left.angle + right.angle) / 2.0f; can_align = true; 
        } else if (left.found) { 
            RCLCPP_INFO(node->get_logger(), "Aligning with respect to left wall.");
            error = left.angle; can_align = true; 
        } else if (right.found) { 
            RCLCPP_INFO(node->get_logger(), "Aligning with respect to right wall.");
            error = right.angle; can_align = true; 
        }

        // If no walls found or error is small
        // 0.2 rad = 11.46 degree
        if (!can_align || std::abs(error) < 0.15f) {
            RCLCPP_INFO(node->get_logger(), "Stopping align. Error: %f deg\n", error * 180 / M_PI);
            send_drive(node, 0.0, 0.0);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            break;
        }

        // Proportional controller for in-place rotation
        float k_rot = 2.0f;
        // float speed_turn = k_rot * error;
        // speed_turn = std::clamp(speed_turn, -1.5f, 1.5f);

        update_encoder();

        float enc;
        if (error > 0) {enc = encoder_left;}
        else {enc = encoder_right;}
        
        float dist_turn = r * (abs(enc));
        float dist_desired_turn = error * b;
        float encoder_desired = dist_desired_turn / r;
        float speed_turn = calculate_speed(dist_turn, dist_desired_turn, k_rot);
    
        send_drive(node, -fmax(speed_turn, 2), fmax(speed_turn, 2));
    
        RCLCPP_INFO(node->get_logger(), "Heading Error: %f deg", error * 180 / M_PI);

        if (encoder_desired - enc < 0.003) break;
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Alignment complete.\n");
    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

}
  
void go_straight(std::shared_ptr<rclcpp::Node> node){
    float speed = 0.0; 
    float dist_desired = 0.80;
    float k_straight = 15;
    
    last_time = std::chrono::steady_clock::now(); 
    
    while(rclcpp::ok()) {
        update_encoder();

        double dist = r * encoder_right;

        WallInfo front_wall = check_wall(node, 120.0f, 0.60f, false);
        WallInfo left_wall = check_wall(node, 210.0f, 0.60f, false);
        WallInfo right_wall = check_wall(node, 30.0f, 0.60f, false);

        float lat_error = 0.0;
        float heading_error = 0.0;

        if (left_wall.found && right_wall.found) {
            lat_error = (left_wall.distance - right_wall.distance) / 2.0; 
            heading_error = (left_wall.angle + right_wall.angle) / 2.0;
        } else if (left_wall.found) {
            lat_error = left_wall.distance - HALF_CELL; 
            heading_error = left_wall.angle;
        } else if (right_wall.found) {
            lat_error = HALF_CELL - right_wall.distance; 
            heading_error = right_wall.angle;
        }

        heading_error = heading_error * (-1);

        // Active control based on lateral tracking and heading tracking
        float k_lat = 2.2;
        float k_head = 0.2;
        float correction_lat = k_lat * lat_error;
        float correction_head = k_head * heading_error;
        float correction = correction_lat + correction_head;
        
        if (std::abs(correction) < 0.1) correction = 0.0;

        // RCLCPP_INFO(node->get_logger(), "-----WALL DETECTION-----");
        // RCLCPP_INFO(node->get_logger(), "Time: %.4f", node->get_clock()->now().seconds());
        // if (left_wall.found) RCLCPP_INFO(node->get_logger(), "Left Wall -> dist: %f, angle: %f", left_wall.distance, left_wall.angle * 180 / M_PI);
        // else RCLCPP_INFO(node->get_logger(), "No Left Wall");
        // if (right_wall.found) RCLCPP_INFO(node->get_logger(), "Right Wall -> dist: %f, angle: %f", right_wall.distance, right_wall.angle * 180 / M_PI);
        // else RCLCPP_INFO(node->get_logger(), "No Right Wall");

        // RCLCPP_INFO(node->get_logger(), "correction: %f", correction);
        // RCLCPP_INFO(node->get_logger(), "    lateral correction: %f, heading correction: %f", correction_lat, correction_head);
        // RCLCPP_INFO(node->get_logger(), "----------------------\n");

        speed = calculate_speed(dist, dist_desired, k_straight);

        // send_drive(node, fmax(speed, 2.5) - correction, fmax(speed, 2.5) + correction);
        send_drive(node, fmax(speed, 2.5) - correction_lat, fmax(speed, 2.5) + correction_lat);

        if (dist_desired - dist <= 0.005) {
            break;
        }

        if (front_wall.found && front_wall.distance + delta < 0.37) {
            break;
        }

        rate.sleep();
    }

    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
}

void turn_180(std::shared_ptr<rclcpp::Node> node) {
    // align_heading(node);
    RCLCPP_INFO(node->get_logger(), "\n\nturning 180 deg"); 

    float speed_turn = 0.0; 
    float dist_desired_turn = M_PI_2 * b;
    float encoder_desired = dist_desired_turn / r;
    float k_turn = 20;

    last_time = std::chrono::steady_clock::now(); 

    while(rclcpp::ok()) {
        update_encoder();
        double dist_turn = r * (abs(encoder_right));
        speed_turn = calculate_speed(dist_turn, dist_desired_turn, k_turn);

        send_drive(node, -fmax(speed_turn, 2), fmax(speed_turn, 2));

        if (encoder_desired - encoder_right < 0.003) break;
        rate.sleep();
    }
    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    // align_heading(node);
}

void turn_90_CW(std::shared_ptr<rclcpp::Node> node) {
    // align_heading(node);
    RCLCPP_INFO(node->get_logger(), "\n\nturning 90 deg clockwise"); 

    float speed_turn = 0.0; 
    float dist_desired_turn = M_PI_4 * b;
    float encoder_desired = dist_desired_turn / r;
    float k_turn = 22;

    last_time = std::chrono::steady_clock::now(); 

    while(rclcpp::ok()) {
      update_encoder();
      double dist_turn = r * (abs(encoder_left));
      speed_turn = calculate_speed(dist_turn, dist_desired_turn, k_turn);

      send_drive(node, fmax(speed_turn, 2), -fmax(speed_turn, 2));

      if (encoder_desired - encoder_left < 0.003) break;
      rate.sleep();
    }
    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    // align_heading(node); 
}

void turn_90_CCW(std::shared_ptr<rclcpp::Node> node) {
    // align_heading(node);
    RCLCPP_INFO(node->get_logger(), "\n\nturning 90 deg counterclockwise"); 

    float speed_turn = 0.0; 
    float dist_desired_turn = M_PI_4 * b;
    float encoder_desired = dist_desired_turn / r;
    float k_turn = 22;

    last_time = std::chrono::steady_clock::now(); 

    while(rclcpp::ok()) {
      update_encoder();
      double dist_turn = r * (abs(encoder_right));
      speed_turn = calculate_speed(dist_turn, dist_desired_turn, k_turn);

      send_drive(node, -fmax(speed_turn, 2), fmax(speed_turn, 2));

      if (encoder_desired - encoder_right < 0.003) break;
      rate.sleep();
    }
    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    // align_heading(node);
}

void follow_plan(const std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Request> request,
    std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Response> response, std::shared_ptr<rclcpp::Node> node)
{
    std::vector<int> plan = request->plan;

    reset_encoder();
    int i = 1;

    for(int dir : plan) {
        std::string plan_dir;
        if(dir == 0) plan_dir = "right";
        if(dir == 1) plan_dir = "up";
        if(dir == 2) plan_dir = "left";
        if(dir == 3) plan_dir = "down";

        RCLCPP_INFO(node->get_logger(), "step %i: %s", i, plan_dir.c_str());
        i++;

        // align_heading(node);

        if(dir == state){
            reset_encoder();
            go_straight(node);
        }
        else {
            if(state == 0) {
                if(dir == 1) { reset_encoder(); turn_90_CCW(node); }
                else if(dir == 2) { reset_encoder(); turn_180(node); }
                else if(dir == 3) { reset_encoder(); turn_90_CW(node); }
            }
            else if(state == 1){
                if(dir == 0) { reset_encoder(); turn_90_CW(node); }
                else if(dir == 2) { reset_encoder(); turn_90_CCW(node); }
                else if(dir == 3) { reset_encoder(); turn_180(node); }
            }
            else if(state == 2){
                if(dir == 0) { reset_encoder(); turn_180(node); }
                else if(dir == 1) { reset_encoder(); turn_90_CW(node); }
                else if(dir == 3) { reset_encoder(); turn_90_CCW(node); }
            }
            else if(state == 3){
                if(dir == 0) { reset_encoder(); turn_90_CCW(node); }
                else if(dir == 1) { reset_encoder(); turn_180(node); }
                else if(dir == 2) { reset_encoder(); turn_90_CW(node); }
            }
            reset_encoder();
            go_straight(node);
        }
        reset_encoder();
        state = dir;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("execute_plan_server");

    auto service_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sensor_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto ransac_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto plan_service = node->create_service<lidar_maze_navigation::srv::ExecutePlan>(
        "execute_plan",
        [&](const std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Request> request,
            std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Response> response) {
            follow_plan(request, response, node);
        },
        rmw_qos_profile_services_default,
        service_cbg
    );
    
    rclcpp::SubscriptionOptions sensor_opts;
    sensor_opts.callback_group = sensor_cbg;

    auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_filtered",
        10,
        [&](const sensor_msgs::msg::LaserScan &msg) {
            laser_callback(msg);
        },
        sensor_opts
    );

    rclcpp::SubscriptionOptions ransac_opts;
    ransac_opts.callback_group = ransac_cbg;

    auto ransac_sub = node->create_subscription<lidar_maze_navigation::msg::Ransac>(
    "ransac", 10,
    [&](const lidar_maze_navigation::msg::Ransac &msg) {
        ransac_callback(msg);
    },
    ransac_opts); 
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
