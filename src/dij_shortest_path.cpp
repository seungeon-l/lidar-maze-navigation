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
#include "lidar_maze_navigation/srv/dijkstra.hpp"
#include "lidar_maze_navigation/msg/point.hpp"
#include "lidar_maze_navigation/msg/point_array.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <chrono>
#include <queue>
#include <fstream>
#include <iostream>


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

// Dijkstra variables: 4 nodes per cell - 1 for each incoming direction
const int DR[4]          = { 0, -1,  0, +1 };
const int DC[4]          = { +1,  0, -1,  0 };
const int OPPOSITE[4]    = { 2,   3,  0,   1 };  // opposite of each direction
const int WALL_FOR_DIR[4]= { 0,   1,  2,   3 };  // RIGHT=0,TOP=1,LEFT=2,BOTTOM=3
const float TURN_COST = 0.5;

int num_rows;
int num_cols;

int initial_col;
int initial_row;
int initial_orientation;
int goal_col;
int goal_row;

int start_id;
int goal_id;

bool maps_initialized = false;

// Dijkstra variables

std::vector<double> dist;
std::vector<int> prev;

// A vertex is just an integer id (0 .. V-1).
// adj[v] holds all outgoing edges from v as (neighbour, weight) pairs.
struct Edge {
    int to;
    double weight;
    int orientation;
};

using Graph = std::vector<std::vector<Edge>>;

enum Wall { RIGHT = 0, TOP = 1, LEFT = 2, BOTTOM = 3 };

//callbacks
void map_callback(const lidar_maze_navigation::msg::Grid &msg) {
    map = msg.rows;
}

void pose_callback(const lidar_maze_navigation::msg::Pose &msg) {
    initial_col = msg.column;
    initial_row = msg.row;
    initial_orientation = msg.orientation;
}


bool has_wall(const std::vector<int>& walls, int w) {
    return std::find(walls.begin(), walls.end(), w) != walls.end();
}

inline int cell_id(int row, int col) {
    return row * num_cols + col;
}

inline int state_id(int row, int col, int dir) {
    return dir * (num_rows * num_cols) + row * num_cols + col;
}

inline void decode_cell(int id, int &row, int &col, int &dir) {
    int cells = num_rows * num_cols;
    dir  = id / cells;
    int flat = id % cells;
    row  = flat / num_cols;
    col  = flat % num_cols;
}


// turn map into a graph
// Grid coords: (0,0) is top-left. row increases downward, col increases rightward.
//
// give every cell a unique id: id = row * num_cols + col.
// add an edge between two neighbours when NEITHER side has a wall between them
Graph create_graph() {
    num_rows = map.size();
    num_cols = map[0].cells.size();

    Graph graph(4 * num_rows * num_cols);

    for (int r = 0; r < num_rows; r++) {
        for (int c = 0; c < num_cols; c++) {
            const std::vector<int>& walls = map[r].cells[c].walls;

            for(int in_dir = 0; in_dir < 4; in_dir ++) {
                int from = state_id(r, c, in_dir);

                for(int out_dir = 0; out_dir < 4; out_dir ++) {
                    int nr = r + DR[out_dir];
                    int nc = c + DC[out_dir];

                    if (has_wall(walls, WALL_FOR_DIR[out_dir]))
                        continue;
                    if (has_wall(map[nr].cells[nc].walls, WALL_FOR_DIR[OPPOSITE[out_dir]]))
                        continue;

                    double cost = 1.0;
                    if(in_dir != out_dir) {
                        cost += TURN_COST;
                    }
                    graph[from].push_back({ state_id(nr, nc, out_dir), cost });
                }
            }

            // neighbour to the RIGHT (same row, col+1)
            if (c + 1 < num_cols && !has_wall(walls, RIGHT)
                && !has_wall(map[r].cells[c + 1].walls, LEFT)) {
                graph[cell_id(r, c)].push_back({cell_id(r, c + 1), 1.0});
            }
            // neighbour BELOW (row+1, same col) — that's the BOTTOM direction
            if (r + 1 < num_rows && !has_wall(walls, BOTTOM)
                && !has_wall(map[r + 1].cells[c].walls, TOP)) {
                graph[cell_id(r, c)].push_back({cell_id(r + 1, c), 1.0});
            }
            // neighbour to the LEFT (same row, col-1)
            if (c - 1 >= 0 && !has_wall(walls, LEFT)
                && !has_wall(map[r].cells[c - 1].walls, RIGHT)) {
                graph[cell_id(r, c)].push_back({cell_id(r, c - 1), 1.0});
            }
            // neighbour ABOVE (row-1, same col) — that's the TOP direction
            if (r - 1 >= 0 && !has_wall(walls, TOP)
                && !has_wall(map[r - 1].cells[c].walls, BOTTOM)) {
                graph[cell_id(r, c)].push_back({cell_id(r - 1, c), 1.0});
            }
            
            if(r == initial_row && c == initial_col) {
                start_id = cell_id(r, c);
            }
            if(r == goal_row && c == goal_col) {
                goal_id = cell_id(r, c);
            }
        }
    }
    return graph;
}


// Dijkstra
// Returns dist[] (shortest distance from source to every vertex) and fills
// prev[] (the predecessor of each vertex on its shortest path) so we can
// reconstruct the actual path afterwards.
const double INF = std::numeric_limits<double>::infinity();

void dijkstra(const Graph& graph) {
    int V = graph.size();

    // for each vertex v: dist[v] = inf, prev[v] = none(-1)
    dist.assign(V, INF);
    prev.assign(V, -1);
    std::vector<bool> explored(V, false);   // "set all vertices to unexplored"

    // dist[source] = 0
    int initial = state_id(initial_row, initial_col, initial_orientation);
    dist[initial] = 0.0;

    // Min-heap ordered by distance. std::priority_queue is a MAX-heap by
    // default, so we use std::greater to flip it into a MIN-heap.
    // Each entry is (distance, vertex).
    using Node = std::pair<double, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    pq.push({0.0, initial});

    // while there are still reachable, unexplored vertices
    while (!pq.empty()) {
        // v = least-valued unexplored vertex  (heap gives this in O(log V))
        auto [d, v] = pq.top();
        pq.pop();

        // LAZY DELETION: if we already finalized v, this is a stale entry. Skip.
        // (Equivalently: if d > dist[v], a better path was already found.)
        if (explored[v]) continue;

        // set v to explored
        explored[v] = true;

        // for each edge (v, w):
        for (const Edge& e : graph[v]) {
            int w = e.to;
            double len = e.weight;

            // if dist[v] + len(v, w) < dist[w]:
            if (!explored[w] && dist[v] + len < dist[w]) {
                dist[w] = dist[v] + len;   // dist[w] = dist[v] + len(v, w)
                prev[w] = v;               // prev[w] = v
                pq.push({dist[w], w});     // push the improved distance (may leave a stale copy behind)
            }
        }
    }
}


void compute_shortest_path(std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Request> request,
        std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Response> response, std::shared_ptr<rclcpp::Node> node) 
{
    std::vector<int> path;
    std::vector<int> path_rows;
    std::vector<int> path_cols;
    std::vector<bool> path_turns;

    goal_row = request->goal_row;
    goal_col = request->goal_column;

    if (map.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Map not yet received, cannot compute path.");
        response->path_rows    = {};
        response->path_columns = {};
        return;
    }


    Graph graph = create_graph();
    dijkstra(graph);

    int best_goal_state = -1;
    double best_cost = INF;

    for (int d = 0; d < 4; d++) {
        int sid = state_id(goal_row, goal_col, d);
        if (dist[sid] < best_cost) {
            best_cost       = dist[sid];
            best_goal_state = sid;
        }
    }
 
    if (best_goal_state == -1 || best_cost >= INF) {
        RCLCPP_WARN(node->get_logger(),
            "Goal (%d, %d) is unreachable from (%d, %d).",
            goal_row, goal_col, initial_row, initial_col);
        response->path_rows    = {};
        response->path_columns = {};
        return;
    }

    // invert path
    for (int at = best_goal_state; at != -1; at = prev[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    // turn the path back into (r, c) notation
    int last_cell = -1;
    int i = 0;

    while (i < static_cast<int>(path.size())) {

        int row, col, dir;
        decode_cell(path[i], row, col, dir);

        int cell = row * num_cols + col;

        path_rows.push_back(row);
        path_cols.push_back(col);

        bool turn_here = false;

        // Examine all consecutive states belonging to this cell.
        int j = i;
        int first_dir = dir;
        int last_dir  = dir;

        while (j + 1 < static_cast<int>(path.size())) {

            int nr, nc, ndir;
            decode_cell(path[j + 1], nr, nc, ndir);

            int next_cell = nr * num_cols + nc;

            if (next_cell != cell)
                break;

            last_dir = ndir;
            j++;
        }

        // If orientation changed while remaining in the same cell,
        // Dijkstra inserted a turn here.
        turn_here = (first_dir != last_dir);

        path_turns.push_back(turn_here);

        i = j + 1;
    }

    response->path_rows = path_rows;
    response->path_columns = path_cols;
    response->path_turns = path_turns;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("shortest_path");
    
    // create multiple callback groups
    auto map_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pose_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto dij_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create dijkstra service for shortest path
    auto dij_srv = node->create_service<lidar_maze_navigation::srv::Dijkstra>(
    "shortest_path", 
    [&](const std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Request> request,
        std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Response> response) {
        compute_shortest_path(request, response, node);
    },
    rmw_qos_profile_services_default,
    dij_cbg);
    
    // map_subscriber
    rclcpp::SubscriptionOptions map_opts;
    map_opts.callback_group = map_cbg;
    auto map_sub = node->create_subscription<lidar_maze_navigation::msg::Grid>(
    "map", 10,
    [&](const lidar_maze_navigation::msg::Grid &msg) {
        map_callback(msg);
    },
    map_opts); 

    // pose subscriber
    rclcpp::SubscriptionOptions pose_opts;
    pose_opts.callback_group = pose_cbg;
    auto pose_sub = node->create_subscription<lidar_maze_navigation::msg::Pose>(
    "pose", 10,
    [&](const lidar_maze_navigation::msg::Pose &msg) {
        pose_callback(msg);
    },
    pose_opts);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
