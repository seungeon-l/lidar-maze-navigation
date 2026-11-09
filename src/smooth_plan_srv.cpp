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

// Robot geometry
const float r = 0.032;
const float b = 0.261;
const float delta = 0.128;

const float circle_rad = 0.4; //radius of turning circle; 0.15 < circle_rad < 0.55, so that the robot doesn't hit any walls
const float cell_length = 0.8;
const float HALF_CELL = 0.4;

const float speed_turn = 5;
const float arc_turn = circle_rad * M_PI_2;

// our encoder sim
float encoder_left = 0.0;
float encoder_right = 0.0;
float current_cmd_left = 0.0;
float current_cmd_right = 0.0;
std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

bool TURN_INCOMING = false;
bool JUST_TURNED = false;
bool FIRST_COMMAND = false;
bool LAST_COMMAND = false;
bool TURN_180 = false;

struct Point {
    float x;
    float y;
};

struct Straight {
    int start;
    int nr_cells;
};

std::vector<int> plan;

sensor_msgs::msg::LaserScan latest_scan;
bool scan_received = false;

std::vector<lidar_maze_navigation::msg::Line> walls;


// current heading state
int state = 1;

// Callbacks
void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
    latest_scan = msg;
    scan_received = true;
}

void ransac_callback(const lidar_maze_navigation::msg::Ransac &msg) {
    walls = msg.walls;
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
    encoder_left = 0.0;
    encoder_right = 0.0;
    last_time = std::chrono::steady_clock::now();
}


// Helpers
float threshold = 0.17f;

struct WallInfo {
    bool found;
    float distance;
};

WallInfo check_wall(float center_degree, float threshold = 0.6f) {
    WallInfo result = {false, -1.0f};

    if (!scan_received) return result;
    
    int total = latest_scan.ranges.size();
    if (total == 0) return result;

    float lookup_degree = 40.0f;
    float lookup_index_half = total * (lookup_degree / 2.0f / 240.0f);

    int scan_center = (int)(total * (center_degree / 240.0f)); 
    int scan_start = std::max(0, scan_center - (int)lookup_index_half);
    int scan_end = std::min(total - 1, scan_center + (int)lookup_index_half);

    int total_rays_in_cone = scan_end - scan_start + 1;
    if (total_rays_in_cone <= 0) return result;

    int hit_count = 0;
    float min_dist = std::numeric_limits<float>::infinity();

    for (int i = scan_start; i <= scan_end; i++) {
        float r = latest_scan.ranges[i];
        
        if (std::isfinite(r) && r > latest_scan.range_min && r < latest_scan.range_max) {
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

bool wall_in_front(std::shared_ptr<rclcpp::Node> node) {
    if (!scan_received) return false;
    int total = latest_scan.ranges.size();
    int front_half = (int)(10.0 / (240.0 / total));  // 20 degree scan
    int front_start = total / 2 - front_half;
    int front_end   = total / 2 + front_half;

    for (int i = front_start; i <= front_end; i++) {
        float r = latest_scan.ranges[i];
        if (std::isfinite(r) && r < threshold && r > 0.10){
            RCLCPP_INFO(node->get_logger(), "wall in forn: %f", r);
            return true;
        } 
    }
    return false;
}

bool side_wall_parallel(float tol_deg = 5.0f) {
    float tol = tol_deg * M_PI / 180.0f;
    for (const auto &w : walls) {
        float diff_left  = std::fabs(w.angle - M_PI_2);
        float diff_right = std::fabs(w.angle - 3.0f * M_PI_2);
        if (diff_left < tol || diff_right < tol) {
            return true;
        }
    }
    return false;
}

// Returns the distance to a wall directly in front of the robot
// (normal pointing forward, angle near 0 within tol_deg).
// Returns -1.0 if no such wall is found.
float front_wall_distance(std::shared_ptr<rclcpp::Node> node, float tol_deg = 20.0f) {
    float tol = tol_deg * M_PI / 180.0f;
    float min_dist = -1.0f;
    for (const auto &w : walls) {
        float diff_zero = std::fabs(w.angle);
        float diff_2pi  = std::fabs(w.angle - 2.0f * M_PI);
        RCLCPP_INFO(node->get_logger(), "#############################################");
        RCLCPP_INFO(node->get_logger(), "front wall distance dist %f m", w.distance);
        RCLCPP_INFO(node->get_logger(), "#############################################\n");

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

void go_straight(std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Response> response, std::shared_ptr<rclcpp::Node> node, int straight_len, int i){
    rclcpp::Rate rate(100);
    float speed = 0.0; 
    float k_straight = 25;

    float dist_desired;

    // compute length of the first step
    if(FIRST_COMMAND && LAST_COMMAND) {         // just one command -> go in that direction 0.8m
        dist_desired = 0.8;
    }
    else if(TURN_180 && LAST_COMMAND) {        // plan ends in 180 degree turn -> go 0,4m, turn, go 0.8 m
        if(TURN_INCOMING) {
            dist_desired = 0.4;
            TURN_INCOMING = false;
            JUST_TURNED = true;
        }
        else if(JUST_TURNED) {
            dist_desired = 0.8;
            JUST_TURNED = false;
        }
    }
    else if(FIRST_COMMAND || LAST_COMMAND || TURN_180) {    // first command or last commad or 
        dist_desired = 0.4;                                 // 80 turn in the middle of a plan -> go 0.4m
    }
    else{
        dist_desired = 0.8;                                 // otherwise always 0.8m
    }

    // compute rest of the long straight
    if(straight_len > 1) {
        dist_desired += 0.8 * (straight_len - 1);
    }

    RCLCPP_INFO(node->get_logger(), "walking straight for %f m", dist_desired);

    while(rclcpp::ok()) {

        update_encoder();

        double dist = r * encoder_right;

        WallInfo left_wall = check_wall(210.0f, 0.60f);
        WallInfo right_wall = check_wall(30.0f, 0.60f);

        float lat_error = 0.0;

        if(left_wall.found && right_wall.found) {
            lat_error = (left_wall.distance - right_wall.distance) / 2 - 0.0175;
        }
        else if (left_wall.found) {
            lat_error = left_wall.distance - HALF_CELL + 0.0175;
        }
        else if (right_wall.found) {
            lat_error = HALF_CELL - 0.0175 - right_wall.distance;
        }

        float k_lat = 8;
        float correction = k_lat * lat_error;
        if(fabs(correction) < 0.1) correction = 0.0;

        RCLCPP_INFO(node->get_logger(), "-----WALL DETECTION-----");
        RCLCPP_INFO(node->get_logger(), "Time: %.4f", node->get_clock()->now().seconds());
        RCLCPP_INFO(node->get_logger(), "left wall detected: %s", left_wall.found ? "true" : "false");
        if (left_wall.found) {
            RCLCPP_INFO(node->get_logger(), "    dist: %f", left_wall.distance);
        }
        RCLCPP_INFO(node->get_logger(), "right wall detected: %s", right_wall.found ? "true" : "false");
        if (right_wall.found) {
            RCLCPP_INFO(node->get_logger(), "    dist: %f", right_wall.distance);
        }
        RCLCPP_INFO(node->get_logger(), "correction: %f", correction);
        RCLCPP_INFO(node->get_logger(), "----------------------\n");

        speed = fmin(6, calculate_speed(dist, dist_desired, k_straight));
        // speed = calculate_speed(dist, dist_desired, k_straight);

        send_drive(node, fmax(speed, 2.5) - correction, fmax(speed, 2.5) + correction);

        // RCLCPP_INFO(node->get_logger(), "encoder_left: %f", encoder_left);
        // RCLCPP_INFO(node->get_logger(), "encoder_right: %f", encoder_right);
        RCLCPP_INFO(node->get_logger(), "travelled dist: %f", dist);
        // if (wall_in_front(node)) {
        //     send_drive(node, 0.0, 0.0);
        //     RCLCPP_ERROR(node->get_logger(), "Plan is not executable: wall detected in front!");
        //     response->success = false;
        //     return;
        // }
        float front_d = front_wall_distance(node, 20.0f);
        bool wall_stop = (front_d > 0.10f && front_d <= 0.35f);
        RCLCPP_INFO(node->get_logger(), "front wall at %.3f m", front_d);

        if (dist_desired - dist <= 0.005 || wall_stop) {
            if (wall_stop) {
                RCLCPP_ERROR(node->get_logger(), "stopping early: front wall at %.3f m", front_d);
            }
            break;
        }

        //   rclcpp::spin_some(node);
        rate.sleep();
    }

    // send_drive(node, 0.0, 0.0);
    // rclcpp::sleep_for(std::chrono::milliseconds(200));
}



void turn_180(std::shared_ptr<rclcpp::Node> node) {
    rclcpp::Rate rate(100);
    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    RCLCPP_INFO(node->get_logger(), "\n\nturning 180 deg"); 

    float speed_turn = 0.0; 
    float dist_desired_turn = M_PI_2 * b;
    float k_turn = 22;

    while(rclcpp::ok()) {

        update_encoder();

        double dist_turn = r * ((abs(encoder_right) + abs(encoder_left)) / 2);
        speed_turn = calculate_speed(dist_turn, dist_desired_turn, k_turn);

        send_drive(node, -fmax(speed_turn, 2), fmax(speed_turn, 2));

        double angle = (abs(encoder_right)) * r / b;

        RCLCPP_INFO(node->get_logger(), "angle: %f", angle);
        RCLCPP_INFO(node->get_logger(), "----------------------\n");

        if (dist_desired_turn - dist_turn <= 0.003) {
            break;
        }

        // rclcpp::spin_some(node);
        rate.sleep();
    }
}


void turn_90_CW(std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Response> response, std::shared_ptr<rclcpp::Node> node) {
    rclcpp::Rate rate(100);
    RCLCPP_INFO(node->get_logger(), "\n\nturning 90 deg clockwise"); 

    // float speed_turn = 0.0; 
    // float k_turn = 22;

    float rad_left = circle_rad + b / 2;
    float rad_right = circle_rad - b / 2;
    float speed_left = speed_turn * rad_left / circle_rad;
    float speed_right = speed_turn * rad_right / circle_rad;

    WallInfo right_wall = check_wall(30.0f, 0.60f);

    float laser_left = latest_scan.ranges[134];
    float laser_right = latest_scan.ranges[634];

    while(rclcpp::ok()) {

        update_encoder();

        double dist_turn = r * (encoder_left + encoder_right) / 2;

        WallInfo right_wall = check_wall(30.0f, 0.60f);

        float lat_error = 0.0;

        if(right_wall.found) {
            lat_error = HALF_CELL - 0.0175 - right_wall.distance;
        }

        float k_lat = 8;
        float correction = k_lat * lat_error;
        if(abs(correction) < 0.1) correction = 0.0;

        RCLCPP_INFO(node->get_logger(), "-----WALL DETECTION-----");
        RCLCPP_INFO(node->get_logger(), "Time: %.4f", node->get_clock()->now().seconds());    
        RCLCPP_INFO(node->get_logger(), "right wall detected: %s", right_wall.found ? "true" : "false");
        if (right_wall.found) {
            RCLCPP_INFO(node->get_logger(), "    dist: %f", right_wall.distance);
        }
        RCLCPP_INFO(node->get_logger(), "correction: %f", correction);
        RCLCPP_INFO(node->get_logger(), "----------------------\n");

        // speed = fmin(6, calculate_speed(dist, dist_desired, k));
        // speed = calculate_speed(dist, dist_desired, k_straight);

        send_drive(node, fmax(speed_left, 2.5) - correction, fmax(speed_right, 2.5) + correction);

        // RCLCPP_INFO(node->get_logger(), "encoder_left: %f", encoder_left);
        // RCLCPP_INFO(node->get_logger(), "encoder_right: %f", encoder_right);
        // RCLCPP_INFO(node->get_logger(), "travelled dist: %f", dist);

        // send_drive(node, -fmax(speed_turn, 2), fmax(speed_turn, 2));
        // send_drive(node, speed_left, speed_right);

        // double angle = (abs(encoder_right)) * r / b;

        // RCLCPP_INFO(node->get_logger(), "angle: %f", angle);
        // RCLCPP_INFO(node->get_logger(), "----------------------\n");
        if (wall_in_front(node)) {
            send_drive(node, 0.0, 0.0);
            RCLCPP_ERROR(node->get_logger(), "Plan is not executable: wall detected in front!");
            response->success = false;
            return;
        }

        bool near_end_of_arc = dist_turn >= 0.5f * arc_turn;
        if ((arc_turn - dist_turn <= 0.003) || (near_end_of_arc && side_wall_parallel(5.0f))) {
            break;
        }

        //   rclcpp::spin_some(node);
        // rate.sleep();
    }

    // send_drive(node, 0.0, 0.0);
    // rclcpp::sleep_for(std::chrono::milliseconds(200));

}
void turn_90_CCW(std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Response> response, std::shared_ptr<rclcpp::Node> node) {
    rclcpp::Rate rate(100);
    RCLCPP_INFO(node->get_logger(), "\n\nturning 90 deg counterclockwise"); 

    // float speed_turn = 0.0; 
    // float k_turn = 22;

    float rad_left = circle_rad - b / 2;
    float rad_right = circle_rad + b / 2;
    float speed_left = speed_turn * rad_left / circle_rad;
    float speed_right = speed_turn * rad_right / circle_rad;

    float laser_left = latest_scan.ranges[134];
    float laser_right = latest_scan.ranges[634];

    while(rclcpp::ok()) {

        update_encoder();

        double dist_turn = r * (encoder_left + encoder_right) / 2;

        WallInfo left_wall = check_wall(210.0f, 0.60f);

        float lat_error = 0.0;

        if(left_wall.found) {
            lat_error = left_wall.distance - HALF_CELL + 0.0175;
        }

        float k_lat = 8;
        float correction = k_lat * lat_error;
        if(abs(correction) < 0.1) correction = 0.0;

        RCLCPP_INFO(node->get_logger(), "-----WALL DETECTION-----");
        RCLCPP_INFO(node->get_logger(), "Time: %.4f", node->get_clock()->now().seconds());    
        RCLCPP_INFO(node->get_logger(), "right wall detected: %s", left_wall.found ? "true" : "false");
        if (left_wall.found) {
            RCLCPP_INFO(node->get_logger(), "    dist: %f", left_wall.distance);
        }
        RCLCPP_INFO(node->get_logger(), "correction: %f", correction);
        RCLCPP_INFO(node->get_logger(), "----------------------\n");

        // speed = fmin(6, calculate_speed(dist, dist_desired, k));
        // speed = calculate_speed(dist, dist_desired, k_straight);

        send_drive(node, fmax(speed_left, 2.5) - correction, fmax(speed_right, 2.5) + correction);

        // RCLCPP_INFO(node->get_logger(), "encoder_left: %f", encoder_left);
        // RCLCPP_INFO(node->get_logger(), "encoder_right: %f", encoder_right);
        // RCLCPP_INFO(node->get_logger(), "travelled dist: %f", dist);

        // send_drive(node, -fmax(speed_turn, 2), fmax(speed_turn, 2));
        // send_drive(node, speed_left, speed_right);

        // double angle = (abs(encoder_right)) * r / b;

        // RCLCPP_INFO(node->get_logger(), "angle: %f", angle);
        // RCLCPP_INFO(node->get_logger(), "----------------------\n");
        if (wall_in_front(node)) {
            send_drive(node, 0.0, 0.0);
            RCLCPP_ERROR(node->get_logger(), "Plan is not executable: wall detected in front!");
            response->success = false;
            return;
        }

        bool near_end_of_arc = dist_turn >= 0.5f * arc_turn;
        if ((arc_turn - dist_turn <= 0.003) || (near_end_of_arc && side_wall_parallel(5.0f))) {
            break;
        }

        //   rclcpp::spin_some(node);
        // rate.sleep();
    }

    // send_drive(node, 0.0, 0.0);
    // rclcpp::sleep_for(std::chrono::milliseconds(200));
}

void turn_in_place( std::shared_ptr<rclcpp::Node> node, int dir) {
    rclcpp::Rate rate(100);
    if(abs(state - dir) == 2) {  //robot should go in the opposite direction to the one it's facing
        turn_180(node);
    }
    else {
        float speed_in_place = 0.0; 
        float dist_desired_turn = M_PI_4 * b;
        float encoder_desired = dist_desired_turn / r;
        float k_turn = 22;
            
        last_time = std::chrono::steady_clock::now(); 

        if(state - dir == -1 || state - dir == 3){ //robot should turn 90 degrees CCW
            RCLCPP_INFO(node->get_logger(), "\n\nturning in place 90 deg counterclockwise"); 

            last_time = std::chrono::steady_clock::now(); 

            while(rclcpp::ok()) {
                update_encoder();
                double dist_turn = r * (abs(encoder_right));
                speed_in_place = calculate_speed(dist_turn, dist_desired_turn, k_turn);

                send_drive(node, -fmax(speed_in_place, 2), fmax(speed_in_place, 2));

                double angle = (abs(encoder_right)) * r / b;

                if (encoder_desired - encoder_right < 0.003) {
                    break;
                }

                rate.sleep();
            }

            send_drive(node, 0.0, 0.0);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        else if(state - dir == 1 || state - dir == -3) {    //robot should turn 90 degrees CW
            RCLCPP_INFO(node->get_logger(), "\n\nturning in place 90 deg clockwise"); 

            last_time = std::chrono::steady_clock::now(); 

            while(rclcpp::ok()) {

                update_encoder();
                double dist_turn = r * (abs(encoder_left));
                speed_in_place = calculate_speed(dist_turn, dist_desired_turn, k_turn);

                send_drive(node, fmax(speed_in_place, 2), -fmax(speed_in_place, 2));
                double angle = (abs(encoder_left)) * r / b;

                if (encoder_desired - encoder_left < 0.003) {
                    break;
                }

                rate.sleep();
            }

            send_drive(node, 0.0, 0.0);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
    }
}

void follow_plan(const std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Request> request,
    std::shared_ptr<lidar_maze_navigation::srv::ExecutePlan::Response> response, std::shared_ptr<rclcpp::Node> node)
{
    RCLCPP_INFO(node->get_logger(), "Following plan..."); 
    plan = request->plan;

    reset_encoder();
    response->success = true;

    int straight_len;

    std::vector<Straight> straights;

    for(int i = 0; i < plan.size() - 1; i++) {
        if(plan[i] == plan[i + 1]) {
            Straight s;
            s.start = i;
            s.nr_cells = 0;

            while(i < plan.size() - 1 && plan[i] == plan[i + 1]) {
                s.nr_cells ++;
                i ++;
            }
            s.nr_cells++;
            i --;
            straights.push_back(s);
        }
        
    }

    for (int i = 0; i < plan.size(); i++) {
        int dir = plan[i];
        RCLCPP_INFO(node->get_logger(), "state = %i, dir = %i", state, dir); 

        TURN_180 = false;
        straight_len = 0;

        if(i == 0) {
            FIRST_COMMAND = true;
        }
        else{
            FIRST_COMMAND = false;
        }

        if(i == plan.size() - 1) {
            LAST_COMMAND = true;
        }
        else{
            LAST_COMMAND = false;
        }

        
        if(!FIRST_COMMAND && abs(plan[i - 1] - dir) == 2) {
            TURN_INCOMING = true;
        }
        else {
            TURN_INCOMING = false;
        }

        // execute direction
        if (state == dir) {
            for(Straight s : straights) {
                if(i == s.start) {
                    straight_len = s.nr_cells;
                    i += s.nr_cells - 1;
                }
            }
            LAST_COMMAND = (i == (int)plan.size() - 1);
            reset_encoder();
            go_straight(response, node, straight_len, i);
        }
        else {
            if(FIRST_COMMAND) {
                reset_encoder();
                turn_in_place(node, dir);
                reset_encoder();
                go_straight(response ,node, straight_len, i);
            }
            else{
                if(state == 0) {
                    if(dir == 1){
                        reset_encoder();
                        turn_90_CCW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                    else if(dir == 2){
                        reset_encoder();
                        turn_180(node);
                        JUST_TURNED = true;
                        bool next_is_straight = !LAST_COMMAND && (plan[i + 1] == dir);
                        bool next_is_turn_and_last = LAST_COMMAND;
                        if (next_is_straight) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                        else if (next_is_turn_and_last) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                    }
                    else if(dir == 3){
                        reset_encoder();
                        turn_90_CW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                }
                else if(state == 1){
                    if(dir == 0){
                        reset_encoder();
                        turn_90_CW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                    else if(dir == 2){
                        reset_encoder();
                        turn_90_CCW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                    else if(dir == 3){
                        reset_encoder();
                        turn_180(node);
                        JUST_TURNED = true;
                        bool next_is_straight = !LAST_COMMAND && (plan[i + 1] == dir);
                        bool next_is_turn_and_last = LAST_COMMAND;
                        if (next_is_straight) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                        else if (next_is_turn_and_last) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                    }
                }
                else if(state == 2){
                    if(dir == 0){
                        reset_encoder();
                        turn_180(node);
                        JUST_TURNED = true;
                        bool next_is_straight = !LAST_COMMAND && (plan[i + 1] == dir);
                        bool next_is_turn_and_last = LAST_COMMAND;
                        if (next_is_straight) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                        else if (next_is_turn_and_last) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                    }
                    else if(dir == 1){
                        reset_encoder();
                        turn_90_CW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                    else if(dir == 3){
                        reset_encoder();
                        turn_90_CCW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                }
                else if(state == 3){
                    if(dir == 0){
                        reset_encoder();
                        turn_90_CCW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                    else if(dir == 1){
                        reset_encoder();
                        turn_180(node);
                        JUST_TURNED = true;
                        bool next_is_straight = !LAST_COMMAND && (plan[i + 1] == dir);
                        bool next_is_turn_and_last = LAST_COMMAND;
                        if (next_is_straight) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                        else if (next_is_turn_and_last) {
                            reset_encoder();
                            go_straight(response, node, 0, i);
                        }
                    }
                    else if(dir == 2){
                        reset_encoder();
                        turn_90_CW(response, node);
                        if(LAST_COMMAND) {
                            reset_encoder();
                            go_straight(response, node, straight_len, i);
                        }
                    }
                }
            }
            reset_encoder();
        }

        if(response->success == false){
            break;
        }

        state = dir;
    }

    send_drive(node, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("execute_plan_server");
    // auto resetEnc = node->create_client<create_fundamentals::srv::ResetEncoders>("reset_encoders");

    // create two callback groups
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
    service_cbg);
    
    rclcpp::SubscriptionOptions sensor_opts;
    sensor_opts.callback_group = sensor_cbg;

    auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", 10,
    [&](const sensor_msgs::msg::LaserScan &msg) {
        laser_callback(msg);
    },
    sensor_opts); 

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


/*
plan=[1, 2, 3, 3, 3, 1] - n-a mai facut ultimul 1
- reparat i-uri sarite de ++ si for
- reparat mate cu paranteze

plan=[1, 3] - la sf a mers 0,4 si s-a oprit intre celule
    cazuri dif pt turn180 and continue plan si turn180 and end plan
- sper ca am reparat cu just_turned si turn_incoming

plan=[1, 2, 3, 3, 0] - a facut 1233 si s-a oprit la sf celulei
- probabil mi-a sarit iar ultimul i

*/

/*
ros2 service call /execute_plan lidar_maze_navigation/srv/ExecutePlan "{plan: [1,2,3,3,3,1]}"
- straight de lungime 2 - cam wonky si agresiv corectat -> fa-l mai incet pe straight-uri lungi
- 180 turn la sf de tot - s-a intors da n-a mai mers inapoi 80cm -  walking straight for 0.000000 m

incercarea 2
- a mers 1233331 cred? ba nu, ca 1 mai are 0.4 in fata


ros2 service call /execute_plan lidar_maze_navigation/srv/ExecutePlan "{plan: [1,2,0,0]}"
- turn 180 la mijloc 
- bun da verifica ca a facut ultimul 0 -> s-a intors si a mers 40 si 40 iar probabil bun tho
- ac. plan da cu inca un 0 la coada: mica eroare care l-a derutat rau...vezi daca poti sa adaugi un 
'intoarce-te pana esti paralel cu peretii ca sa n-o iei haihui'

incercarea 2
- ok


ros2 service call /execute_plan lidar_maze_navigation/srv/ExecutePlan "{plan: [1,2,3,3,3,3]}"
- straight de lungime 3 (la sf)
- travelled dist: 2.395507
- s-a oprit la sf celulei
- s-a oprit inainte de ultimul pas siii a corectat putin agresiv

incercarea 2
- stark :)



ros2 service call /execute_plan lidar_maze_navigation/srv/ExecutePlan "{plan: [1,1,0,0]}" 
- start mijl. spate
- straight de lungime 1
-okkkk da s-a oprit putin tarziu

incercarea 2
- tot putin tarziu, poate 0.5 in go_straight in loc de 0.45


ros2 service call /execute_plan lidar_maze_navigation/srv/ExecutePlan "{plan: [1,1,1]}"
- doar straight de 3 ori 
- ai si straight lung la inceput asa
- toppppp

incercarea 2
- topppp

ros2 service call /execute_plan lidar_maze_navigation/srv/ExecutePlan "{plan: [1,1,0,0,0,0]}"
- doua straights lungi una dupa alta
- top

ros2 service call /execute_plan lidar_maze_navigation/srv/ExecutePlan "{plan: [1,1,2,3,3,3]}"
- doua straights lungi 
- top



alte probleme: 
- wall in front il vede la lipeala - 
- corectat agresivitatea la straighturi lungi - 
- control 180 turn with parallel walls 
*/
