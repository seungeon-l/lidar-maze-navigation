#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar_maze_navigation/msg/ransac.hpp"
#include "lidar_maze_navigation/msg/grid.hpp"
#include "lidar_maze_navigation/msg/row.hpp"
#include "lidar_maze_navigation/msg/cell.hpp"
#include "lidar_maze_navigation/msg/pose.hpp"
#include "lidar_maze_navigation/srv/potential_field.hpp"
#include "lidar_maze_navigation/msg/exact_pose.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <chrono>
#include <queue>
#include <fstream>


// Robot geometry
const float r = 0.032;
const float b = 0.261;
const float delta = 0.128;

const float circle_rad = 0.4; //radius of turning circle; 0.15 < circle_rad < 0.55, so that the robot doesn't hit any walls
const float cell_length = 0.8;
const float HALF_CELL = 0.4;
const float mini_cell_length = cell_length / 10.0f;
const int scale = 10; 

const float speed_turn = 5;
const float arc_turn = circle_rad * M_PI_2;

// our encoder sim
float encoder_left = 0.0;
float encoder_right = 0.0;
float current_cmd_left = 0.0;
float current_cmd_right = 0.0;
std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

int state;
int prev_state;

sensor_msgs::msg::LaserScan latest_scan;
bool scan_received = false;

std::vector<lidar_maze_navigation::msg::Line> walls;

std::vector<lidar_maze_navigation::msg::Row> map;
std::vector<std::vector<int>> wall_map;         // a value is 1 if mini cell is part of a wall, else 0
std::vector<std::vector<int>> distance_map;     // value is the distance of a mini cell to the nearest wall
std::vector<std::vector<std::vector<std::pair<float, float>>>> pot_field_map;   

std::vector<std::pair<int, int>> path;          // list of mini cell centres (row, col) that the robot will pass through 
std::vector<std::pair<int, int>> sub_goals;     // list of mini cell edges (row, col) that the robot will pass through 

lidar_maze_navigation::msg::Pose initial_pose;

int initial_col;
int initial_row;

bool maps_initialized = false;

std::vector<int> plan;

// potential field parameters
const float k_attr = 1.0f;  // attractive potential gain
const float k_rep = 0.33f;   // repulsive potential gain
const float d0 = 5 * mini_cell_length;  // distance of influence of obstacles in mini cells



// callbacks
void map_callback(const lidar_maze_navigation::msg::Grid &msg) {
    map = msg.rows;
}


void pose_callback(const lidar_maze_navigation::msg::Pose &msg) {
    initial_col = msg.column;
    initial_row = msg.row;
}

// void exact_pose_callback(const lidar_maze_navigation::msg::ExactPose &msg) {
//     current_col = msg.current_col;
//     current_row = msg.current_row;
// }


void mini_cell_to_world_center(int row, int col, float &x, float &y){
    x = (col + 0.5f) * mini_cell_length;
    y = (row + 0.5f) * mini_cell_length;
}

void world_to_mini_cell(float x, float y, int &row, int &col){   
    row = (int)std::floor(y/ mini_cell_length); 
    col = (int)std::floor(x/ mini_cell_length);   //given a particle's continuous world coordinates, returns which grid cell it currently sits in
}


// specific grid: in each big cell, 10x10 mini cells
// on specific grid, save whether a mini cell is part of a wall or not
void initialize_wall_map() {
    int rows = map.size();
    int cols = map[0].cells.size();
    int grid_rows = rows * scale;
    int grid_cols = cols * scale; 
    
    wall_map.assign(grid_rows, std::vector<int>(grid_cols, 0));     // initialize all mini cells as free (0)

    for(int row = 0; row < rows; row++) {
        for(int col = 0; col < cols; col++) {
            for(int wall : map[row].cells[col].walls) {
                
                if(wall == 0) {         // right wall
                    for(int i = 0; i < scale; i++) {
                        wall_map[row * scale + i][(col + 1) * scale - 1] = 1;
                    }
                }
                else if(wall == 1) {    // top wall
                    for(int i = 0; i < scale; i++) {
                        wall_map[row * scale][col * scale + i] = 1;
                    }
                }
                else if(wall == 2) {    // left wall
                    for(int i = 0; i < scale; i++) {
                        wall_map[row * scale + i][col * scale] = 1;
                    }
                }
                else if(wall == 3) {    // bottom wall
                    for(int i = 0; i < scale; i++) {
                        wall_map[(row + 1) * scale - 1][col * scale + i] = 1;
                    }
                }
            }
        }
    }
}


// get the 8 neighbours of a mini cell
std::vector<std::pair<int, int>> get_neighbours(int r, int c) {
    int grid_rows = map.size() * scale;
    int grid_cols = map[0].cells.size() * scale;

    std::vector<std::pair<int, int>> neighbours;
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    for(int i = 0; i < 4; i++) {
        int nr = r + dr[i];
        int nc = c + dc[i];
        
        if(nr >= 0 && nr < grid_rows && nc >= 0 && nc < grid_cols) {    // make sure neighbours are not out of bounds
            neighbours.push_back({nr, nc});
        }
    }

    return neighbours;
}


//brushfire algorithm: get distance to nearest obstacle for each mini cell in the map
void brushfire() {
    int grid_rows = map.size() * scale;
    int grid_cols = map[0].cells.size() * scale; 

    std::queue<std::pair<int, int>> queue;            // queue of mini cells to process   
    distance_map.assign(grid_rows, std::vector<int>(grid_cols, 0));   // initialize all distances to 0
    
    for(int r = 0; r < grid_rows; r++) {   // start with the cells that are part of the walls (distance = 1)
        for(int c = 0; c < grid_cols; c++) {
            if(wall_map[r][c] == 1) {
                distance_map[r][c] = 1;      
                queue.push({r, c});
            }  
        }
    }

    // iterate trhough the map starting with wall cells
    // update the distance of each cell to the nearest wall
    while(!queue.empty()) {
        auto [r, c] = queue.front();
        queue.pop();

        std::vector<std::pair<int, int>> neighbours = get_neighbours(r, c); // get the 4 neighbouring mini-cells of the current cell

        for(auto [nr, nc] : neighbours) {
            if(wall_map[nr][nc] == 0 && distance_map[nr][nc] == 0) {    // if the neighbour hasn't been visited yet
                distance_map[nr][nc] = distance_map[r][c] + 1;   // set the distance of the neighbour to 1 + the distance of the current cell
                queue.push({nr, nc});     // add the neighbour to the queue to process its neighbours later
            }
        }
    }
}



// create potential field based on distance map
void create_potential_field(std::shared_ptr<lidar_maze_navigation::srv::PotentialField::Request> request, std::shared_ptr<lidar_maze_navigation::srv::PotentialField::Response> response, std::shared_ptr<rclcpp::Node> node) {
    // make sure you only initialize the wall_map and distance_map once
    if(!maps_initialized) {
        if(map.empty()) {
            return;
        }
        initialize_wall_map();
        brushfire();
        maps_initialized = true;
    }

    // plan = request->plan;


    // vector to be returned
    float dx = 0.0f;
    float dy = 0.0f;

    pot_field_map.assign(map.size() * scale,
        std::vector<std::vector<std::pair<float,float>>>(
            map[0].cells.size() * scale,
            std::vector<std::pair<float,float>>(2, {0.0f, 0.0f}))); // pre-fill both layers with zero
    


    // repulsive gradient
    for(int r = 0; r < distance_map.size(); r++) {
        for(int c = 0; c < distance_map[0].size(); c++) {
            float dist = distance_map[r][c] * mini_cell_length;   // convert distance from mini cells to meters

            if(dist > 0 && dist < d0) {
                // U = 0.5 * k_rep * (1/dist - 1/d0)^2 - Khatlib potential field
                float dU_dr = 0.0f;
                float dU_dc = 0.0f;

                // initialize all neighbour distanced as dist in case they're out of bounds
                float d_up = dist;
                float d_down = dist;
                float d_left = dist;
                float d_right = dist;

                // get neighbour distances
                if(r - 1 >= 0 && r - 1 < distance_map.size()) {
                    d_up = distance_map[r - 1][c] * mini_cell_length;
                }
                if(r + 1 >= 0 && r + 1 < distance_map.size()) {
                    d_down = distance_map[r + 1][c] * mini_cell_length;
                }
                if(c - 1 >= 0 && c - 1 < distance_map[0].size()) {
                    d_left = distance_map[r][c - 1] * mini_cell_length;
                }
                if(c + 1 >= 0 && c + 1 < distance_map[0].size()) {
                    d_right = distance_map[r][c + 1] * mini_cell_length;
                }

                dU_dr = (d_down - d_up) / (2.0f * mini_cell_length);
                dU_dc = (d_right - d_left) / (2.0f * mini_cell_length);

                // convert scalar potential into force vector
                // (1/dist - 1/d0) = how strong is the repulsion at the current distance
                // * (1/(dist*dist)) = from derivating U
                // sign-change: k_rep might have to be positive
                // repulsive gradient \u2014 fixed stencil + linear ramp + correct sign

                const float k_rep_new = 2.9f;
                const float d0_new    = 0.4f;
                int R = (int)distance_map.size();
                int C = (int)distance_map[0].size();

                for (int r = 0; r < R; r++) {
                    for (int c = 0; c < C; c++) {
                        if (wall_map[r][c] == 1) continue;

                        float dist = distance_map[r][c] * mini_cell_length;
                        if (dist <= 0.0f || dist >= d0_new) continue;

                        // one-sided diffs: never difference across a wall cell
                        bool upf  = (r-1 >= 0) && wall_map[r-1][c] == 0;
                        bool dnf  = (r+1 <  R) && wall_map[r+1][c] == 0;
                        bool lff  = (c-1 >= 0) && wall_map[r][c-1] == 0;
                        bool rtf  = (c+1 <  C) && wall_map[r][c+1] == 0;

                        float du = upf ? distance_map[r-1][c]*mini_cell_length : dist;
                        float dd = dnf ? distance_map[r+1][c]*mini_cell_length : dist;
                        float dl = lff ? distance_map[r][c-1]*mini_cell_length : dist;
                        float dr_ = rtf ? distance_map[r][c+1]*mini_cell_length : dist;

                        float dU_dc, dU_dr;
                        if (lff && rtf)  dU_dc = (dr_ - dl) / (2.0f*mini_cell_length);
                        else if (rtf)    dU_dc = (dr_ - dist) / mini_cell_length;
                        else if (lff)    dU_dc = (dist - dl)  / mini_cell_length;
                        else             dU_dc = 0.0f;

                        if (upf && dnf)  dU_dr = (dd - du) / (2.0f*mini_cell_length);
                        else if (dnf)    dU_dr = (dd - dist) / mini_cell_length;
                        else if (upf)    dU_dr = (dist - du) / mini_cell_length;
                        else             dU_dr = 0.0f;

                        float gmag = std::hypot(dU_dc, dU_dr);
                        if (gmag < 1e-6f) continue;

                        float mag = k_rep_new * (1.0f - dist / d0_new);   // linear ramp, not 1/d^2

                        // NEGATED: this is the fix for "all arrows point into walls"
                        float fx = mag * dU_dc / gmag;
                        float fy = -mag * dU_dr / gmag;

                        pot_field_map[r][c][0].first  += fx;
                        pot_field_map[r][c][0].second += fy;
                        pot_field_map[r][c][1].first  += fx;
                        pot_field_map[r][c][1].second += fy;
                    }
                }

                std::pair<float, float> force_vec = {dx, dy};
                pot_field_map[r][c][0].first += force_vec.first;
                pot_field_map[r][c][0].second += force_vec.second;
                pot_field_map[r][c][1].first += force_vec.first;
                pot_field_map[r][c][1].second += force_vec.second;
            } 
        }
    }

    int grid_rows = map.size() * scale;
    int grid_cols = map[0].cells.size() * scale;
    response->rows = grid_rows;
    response->cols = grid_cols;
    response->gx.assign(2 * grid_rows * grid_cols, 0.0f);
    response->gy.assign(2 * grid_rows * grid_cols, 0.0f);
    for (int f = 0; f < 2; f++){
        for (int rr = 0; rr < grid_rows; rr++){
            for (int cc = 0; cc < grid_cols; cc++) {
                int idx = f * grid_rows * grid_cols + rr * grid_cols + cc;
                response->gx[idx] = pot_field_map[rr][cc][f].first;   // note [r][c][f]
                response->gy[idx] = pot_field_map[rr][cc][f].second;
            }
        }
    }

    std::ofstream pf_file("src/lidar_maze_navigation/pot_field.csv");
    pf_file << "row,col,gx0,gy0\n";
    for (int rr = 0; rr < grid_rows; rr++) {
        for (int cc = 0; cc < grid_cols; cc++) {
            pf_file << rr << "," << cc << ","
                    << pot_field_map[rr][cc][0].first << ","
                    << pot_field_map[rr][cc][0].second << "\n";
        }
    }
    pf_file.close();
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("potential_field");  // was localization
    

    // create multiple callback groups
    auto map_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // auto exact_pose_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pose_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pot_field_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create potential field service
    auto potential_field_srv = node->create_service<lidar_maze_navigation::srv::PotentialField>(
    "potential_field", 
    [&](const std::shared_ptr<lidar_maze_navigation::srv::PotentialField::Request> request,
        std::shared_ptr<lidar_maze_navigation::srv::PotentialField::Response> response) {
        create_potential_field(request, response, node);
    },
    rmw_qos_profile_services_default,
    pot_field_cbg);
    
    // map_subscriber
    rclcpp::SubscriptionOptions map_opts;
    map_opts.callback_group = map_cbg;
    auto map_sub = node->create_subscription<lidar_maze_navigation::msg::Grid>(
    "map", 10,
    [&](const lidar_maze_navigation::msg::Grid &msg) {
        map_callback(msg);
    },
    map_opts); 

    // // specific pose subscriber
    // rclcpp::SubscriptionOptions exact_pose_opts;
    // exact_pose_opts.callback_group = exact_pose_cbg;
    // auto exact_pose_sub = node->create_subscription<lidar_maze_navigation::msg::ExactPose>(
    // "exact_pose", 10,
    // [&](const lidar_maze_navigation::msg::ExactPose &msg) {
    //     exact_pose_callback(msg);
    // },
    // exact_pose_opts);

    // pose subscriber
    rclcpp::SubscriptionOptions pose_opts;
    pose_opts.callback_group = pose_cbg;
    auto pose_sub = node->create_subscription<lidar_maze_navigation::msg::Pose>(
    "pose", 10,
    [&](const lidar_maze_navigation::msg::Pose &msg) {
        pose_callback(msg);
    },
    pose_opts);

    // create_potential_field(potential_field_pub, node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}


// TODO:
// sometimes it doesn't stop    -- changed check_if_lost function (still needs to be tested)
// fix subgoals                 -- done, should be fine          
// wrong heading sometimes? 
// do particle weights during localization grow when certain? or do we just not update the plotter? -- it was the plotter lol


// current stand:
// 1. localization works well
// first plan after localization: was supposed to turn 180 degrees in place but did not (went fwd into the wall instead)
// after that, the heading was messed up cause i moved him and he didn't accept any plan
// after each new plan, he would immediately declare he's   lost and relocalize
// relocalization worked pretty well most of the times: 3/4 tries were correct

// same code, this time with a 180 turn in the plan:
// he often goes off in the wrong direction when starting a plan
