#include "rclcpp/rclcpp.hpp"
#include "lidar_maze_navigation/msg/pose.hpp"
#include <vector>
#include <cmath>
#include <memory>

struct Particle {
    float x;    
    float y;
    float angle;
    float weight;
};

extern std::vector<Particle> particle_map;


// Declaration: tell the compiler this function exists somewhere
void run_localization( std::shared_ptr<rclcpp::Node> node, rclcpp::Publisher<lidar_maze_navigation::msg::Pose>::SharedPtr pose_pub);
