#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "lidar_maze_navigation/srv/execute_plan.hpp" //
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("plan_executor_client");


    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("execute plan");
    rclcpp::Client<lidar_maze_navigation::srv::ExecutePlan>::SharedPtr client =
    node->create_client<lidar_maze_navigation::srv::ExecutePlan>("add_two_ints");

    auto request = std::make_shared<lidar_maze_navigation::srv::ExecutePlan::Request>();

    request-> plan = atoll(argv[1]);
    while(!lidar_maze_navigation::srv::ExecutePlan->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_INFO(node->get_logger(), "waiting for service..");
    }
}
