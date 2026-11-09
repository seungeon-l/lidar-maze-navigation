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

std::vector<std::pair<int,int>> golds;
std::vector<std::pair<int,int>> pickups;
std::vector<std::pair<int,int>> collected_golds;
bool golds_received   = false;
bool pickups_received = false;

struct LegResult {
    double cost = std::numeric_limits<double>::infinity();
    std::vector<std::pair<int,int>> cells;   // (row,col) from source POI .. target POI
};

struct TourResult {
    std::vector<int> order;                       // visiting order, as indices into the golds vector
    std::vector<std::pair<int,int>> path_cells;   // full concatenated (row,col) path
    double cost = std::numeric_limits<double>::infinity();
    bool ok = false;
};

using Graph = std::vector<std::vector<Edge>>;

enum Wall { RIGHT = 0, TOP = 1, LEFT = 2, BOTTOM = 3 };

//callbacks
void map_callback(const lidar_maze_navigation::msg::Grid &msg) {
    map = msg.rows;
}

void gold_callback(const lidar_maze_navigation::msg::PointArray &msg) {
    golds.clear();
    for (const auto &p : msg.points)
        golds.push_back({ p.row, p.col });
    golds_received = true;
}

void pickup_callback(const lidar_maze_navigation::msg::PointArray &msg) {
    pickups.clear();
    for (const auto &p : msg.points)
        pickups.push_back({ p.row, p.col });
    pickups_received = true;
}

void gold_collected_callback(const lidar_maze_navigation::msg::Point &msg) {
    std::pair<int, int> collected = {msg.row, msg.col};
    if (std::find(collected_golds.begin(), collected_golds.end(), collected) == collected_golds.end()) {
        collected_golds.push_back(collected);
    }
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

// Dijkstra that returns its own dist/prev (from a given source)
static void dijkstra_from(const Graph& graph, int source,
                        std::vector<double>& dist, std::vector<int>& prev)
{
    const double LINF = std::numeric_limits<double>::infinity();
    int V = (int)graph.size();
    dist.assign(V, LINF);
    prev.assign(V, -1);
    std::vector<bool> explored(V, false);

    dist[source] = 0.0;
    using Node = std::pair<double,int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    pq.push({0.0, source});

    while (!pq.empty()) {
        auto [d, v] = pq.top(); pq.pop();
        if (explored[v]) continue;
        explored[v] = true;
        for (const Edge& e : graph[v]) {
            if (!explored[e.to] && dist[v] + e.weight < dist[e.to]) {
                dist[e.to] = dist[v] + e.weight;
                prev[e.to] = v;
                pq.push({dist[e.to], e.to});
            }
        }
    }
}

// Build the full ordered POI x POI matrix of (cost, cell-path).
// For each source POI we run Dijkstra from its 4 orientations ONCE and read
// off every target from those 4 dist arrays. 4 runs per POI.
static std::vector<std::vector<LegResult>>
build_leg_matrix(const Graph& g, const std::vector<std::pair<int,int>>& poi)
{
    const double LINF = std::numeric_limits<double>::infinity();
    int P = (int)poi.size();
    std::vector<std::vector<LegResult>> leg(P, std::vector<LegResult>(P));

    for (int i = 0; i < P; i++) {
        std::vector<std::vector<double>> D(4);
        std::vector<std::vector<int>>    Pr(4);
        for (int so = 0; so < 4; so++)
            dijkstra_from(g, state_id(poi[i].first, poi[i].second, so), D[so], Pr[so]);

        for (int j = 0; j < P; j++) {
            if (i == j) continue;
            LegResult best;                // cost = +inf by default
            int best_so = -1, best_arr = -1;

            for (int so = 0; so < 4; so++)
                for (int eo = 0; eo < 4; eo++) {
                    int sid = state_id(poi[j].first, poi[j].second, eo);
                    if (D[so][sid] < best.cost) {
                        best.cost = D[so][sid];
                        best_so   = so;
                        best_arr  = sid;
                    }
                }

            if (best_so >= 0 && std::isfinite(best.cost)) {
                // reconstruct, collapsing consecutive states that share a cell
                std::vector<std::pair<int,int>> cells;
                int last_cell = -1;
                for (int at = best_arr; at != -1; at = Pr[best_so][at]) {
                    int r, c, dir; decode_cell(at, r, c, dir);
                    int cell = r * num_cols + c;
                    if (cell != last_cell) { cells.push_back({r, c}); last_cell = cell; }
                }
                std::reverse(cells.begin(), cells.end());   // now source .. target
                best.cells = std::move(cells);
            }
            leg[i][j] = std::move(best);
        }
    }
    return leg;
}

// Open-path TSP: start fixed at index 0, permute the golds, minimise sum.
// append nearest pickup spot afterwards
TourResult plan_gold_tour(const Graph& g,
                            std::pair<int,int> start,
                            const std::vector<std::pair<int,int>>& golds,
                            const std::vector<std::pair<int,int>>& pickups = {})
{
    const double LINF = std::numeric_limits<double>::infinity();
    TourResult res;
    int n = (int)golds.size();
    if (n == 0) return res;

    // POI 0 = start, 1..n = golds
    std::vector<std::pair<int,int>> poi;
    poi.push_back(start);
    for (auto& gc : golds) poi.push_back(gc);

    auto leg = build_leg_matrix(g, poi);

    std::vector<int> perm(n);
    for (int i = 0; i < n; i++) perm[i] = i + 1;   // gold POI indices 1..n
    std::sort(perm.begin(), perm.end());

    double best_cost = LINF;
    std::vector<int> best_perm;

    do {
        double c = leg[0][perm[0]].cost;           // start -> first gold
        bool ok = std::isfinite(c);
        for (int k = 0; ok && k + 1 < n; k++) {
            double lc = leg[perm[k]][perm[k + 1]].cost;
            if (!std::isfinite(lc)) { ok = false; break; }
            c += lc;
        }
        if (ok && c < best_cost) { best_cost = c; best_perm = perm; }
    } while (std::next_permutation(perm.begin(), perm.end()));

    if (best_perm.empty()) return res;             // some gold unreachable

    // optional: end at the cheapest pickup from the last gold
    LegResult tail;
    if (!pickups.empty()) {
        int last_gold_poi = best_perm.back();
        std::pair<int,int> lg = poi[last_gold_poi];
        // tiny extra matrix row from the last gold to each pickup
        std::vector<std::pair<int,int>> tail_poi = { lg };
        for (auto& p : pickups) tail_poi.push_back(p);
        auto tleg = build_leg_matrix(g, tail_poi);
        double bestp = LINF;
        for (int p = 1; p < (int)tail_poi.size(); p++)
            if (tleg[0][p].cost < bestp) { bestp = tleg[0][p].cost; tail = tleg[0][p]; }
        if (std::isfinite(bestp)) best_cost += bestp;
    }

    // ---- assemble ----
    res.ok    = true;
    res.cost  = best_cost;
    res.order.reserve(n);
    for (int idx : best_perm) res.order.push_back(idx - 1);   // back to gold indices

    auto append_leg = [&](const std::vector<std::pair<int,int>>& cells, bool first) {
        if (cells.empty()) return;
        size_t s = first ? 0 : 1;                 // drop the shared endpoint
        for (size_t k = s; k < cells.size(); k++) res.path_cells.push_back(cells[k]);
    };

    append_leg(leg[0][best_perm[0]].cells, true);
    for (int k = 0; k + 1 < n; k++)
        append_leg(leg[best_perm[k]][best_perm[k + 1]].cells, false);
    if (!tail.cells.empty()) append_leg(tail.cells, false);

    return res;
}


void compute_gold_tour(std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Request> /*request*/,
                        std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Response> response,
                        std::shared_ptr<rclcpp::Node> node)
{
    if (map.empty() || !golds_received) {
        RCLCPP_ERROR(node->get_logger(), "Waiting for map and golds before planning.");
        response->path_rows = {}; response->path_columns = {};
        return;
    }

    // Filter out collected golds
    std::vector<std::pair<int,int>> remaining_golds;
    for (const auto& g : golds) {
        if (std::find(collected_golds.begin(), collected_golds.end(), g) == collected_golds.end()) {
            remaining_golds.push_back(g);
        }
    }

    if (remaining_golds.empty()) {
        RCLCPP_INFO(node->get_logger(), "All golds collected!");
        response->path_rows = {}; response->path_columns = {};
        return;
    }

    Graph g = create_graph();
    std::pair<int,int> start = { initial_row, initial_col };
    TourResult tour = plan_gold_tour(g, start, remaining_golds, pickups);

    if (!tour.ok) {
        RCLCPP_WARN(node->get_logger(), "Gold tour: some gold unreachable.");
        response->path_rows = {}; response->path_columns = {};
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Gold tour cost %.1f, %zu cells",
                tour.cost, tour.path_cells.size());
    // 19june: return only gold locations in order, not the whole path through all of them
    // for (auto &[r, c] : tour.path_cells) {
    //     response->path_rows.push_back(r);
    //     response->path_columns.push_back(c);
    // }
    response->path_turns = {};
    
    for (int idx : tour.order) {
        response->path_rows.push_back(remaining_golds[idx].first);
        response->path_columns.push_back(remaining_golds[idx].second);
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gold_order");
    

    // create multiple callback groups
    auto map_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pose_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto dij_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto gold_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pickup_cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create dijkstra service for shortest path
    auto dij_srv = node->create_service<lidar_maze_navigation::srv::Dijkstra>(
    "gold_order", 
    [&](const std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Request> request,
        std::shared_ptr<lidar_maze_navigation::srv::Dijkstra::Response> response) {
        compute_gold_tour(request, response, node);
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

    // gold subscriber
    rclcpp::SubscriptionOptions gold_opts;
    gold_opts.callback_group = gold_cbg;
    auto gold_sub = node->create_subscription<lidar_maze_navigation::msg::PointArray>(
    "gold", 10,
    [&](const lidar_maze_navigation::msg::PointArray &msg) {
        gold_callback(msg);
    },
    gold_opts);

    // pickup subscriber
    rclcpp::SubscriptionOptions pickup_opts;
    pickup_opts.callback_group = pickup_cbg;
    auto pickup_sub = node->create_subscription<lidar_maze_navigation::msg::PointArray>(
    "pickup", 10,
    [&](const lidar_maze_navigation::msg::PointArray &msg) {
        pickup_callback(msg);
    },
    pickup_opts);

    // gold_collected subscriber
    // We can reuse pickup_cbg or create a new one. Let's just use gold_cbg for simplicity.
    rclcpp::SubscriptionOptions collected_opts;
    collected_opts.callback_group = gold_cbg;
    auto collected_sub = node->create_subscription<lidar_maze_navigation::msg::Point>(
    "gold_collected", 10,
    [&](const lidar_maze_navigation::msg::Point &msg) {
        gold_collected_callback(msg);
    },
    collected_opts);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
