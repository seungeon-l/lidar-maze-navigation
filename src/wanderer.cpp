#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>

using std::placeholders::_1;

enum class directionState {FORWARD, TURNING_LEFT, TURNING_RIGHT};
directionState state = directionState::FORWARD; 

void send_drive(rclcpp::Node::SharedPtr node, float left, float right, auto request, auto diffDrive) {
  request->left = left;
  request->right = right;
  diffDrive->async_send_request(request);

}

void topic_callback(const sensor_msgs::msg::LaserScan &msg, rclcpp::Node::SharedPtr node, auto request, auto diffDrive){

  int total = msg.ranges.size();               // ~683 laser beams in the 240 Degrees range   
  float step_deg = 240.0/ total;             // Variable for degrees per beam ~0.36 Degree per beam 

  // Front: 20 Degree 
  int front_half   = (int)(10.0 / step_deg);  // each side (20 Degrees upfront)
  int front_start  = total / 2 - front_half;
  int front_end    = total / 2 + front_half;

  int front_left_start = front_start - (int)(25.0/ step_deg);  // 25 degrees left of the frontal angle
  int front_left_end = front_start;

  int front_right_start = front_end;
  int front_right_end = front_end + (int)(25.0/ step_deg);  // 25 degrees right of the frontal angle


  // Right: from index 0 up to 80 Degrees worth of beams = ~228
  int right_start  = (int)(10.0/ step_deg);
  int right_end    = (int)(80.0 / step_deg);  // ~228 Beams

  // Left: 80 Degree on the other end, but skip the outermost 10 Degrees (broken side)
  int left_end     = total - 1 - (int)(10.0/ step_deg);  // skip last ~36 beams
  int left_start   = total - 1 - (int)(80.0 / step_deg);

  // valid distances in a range (the beams that actually hit smth.)
  float dist_front = std::numeric_limits<float>::infinity();
  float dist_left  = std::numeric_limits<float>::infinity();
  float dist_right = std::numeric_limits<float>::infinity();
  float dist_front_left = std::numeric_limits<float>::infinity();;
  float dist_front_right = std::numeric_limits<float>::infinity();

  // determine the smallest distance in Range
  for (int i = front_start; i <= front_end; i++) {
    float r = msg.ranges[i];
    if(r <= 0.15 && r > 0.05 && std::isfinite(r)) {
      dist_front = r;
      break;
    }
    if (r > 0.05 && std::isfinite(r) && r < dist_front) dist_front = r;
  }

  for (int i = front_left_start; i <= front_left_end; i++) {
    float r = msg.ranges[i];
    if(r <= 0.20 && r > 0.05 && std::isfinite(r)) {
      dist_front_left = r;
      break;
    }
    if (r > 0.05 && std::isfinite(r) && r < dist_front_left) dist_front_left = r;
  }

  for (int i = front_right_start; i <= front_right_end; i++) {
    float r = msg.ranges[i];
    if(r <= 0.20 && r > 0.05 && std::isfinite(r)) {
      dist_front_right = r;
      break;
    }
    if (r > 0.05 && std::isfinite(r) && r < dist_front_right) dist_front_right = r;
  }

  for (int i = right_start; i <= right_end; i++) {
    float r = msg.ranges[i];
    if (r > 0.05 && std::isfinite(r) && r < dist_right) dist_right = r;
  }

  for (int i = left_start; i <= left_end; i++) {
    float r = msg.ranges[i];
    if (r > 0.05 && std::isfinite(r) && r < dist_left) dist_left = r;
  }

  RCLCPP_INFO(node->get_logger(), "dist left: %f", dist_left);
  RCLCPP_INFO(node->get_logger(), "dist front: %f", dist_front);
  RCLCPP_INFO(node->get_logger(), "dist right: %f", dist_right);


  // case 1: driving forward
  if( state == directionState::FORWARD) {
    if (dist_front <= 0.15 || dist_front_left <= 0.22 || dist_front_right <= 0.22) {                   // distance till hit   
      if (dist_left > dist_right) {             // rotate in the freest direction
          state = directionState::TURNING_LEFT;
      } 
      else {
          state = directionState::TURNING_RIGHT;
      }
    } 
    else {
      send_drive(node, 10.0, 10.0, request, diffDrive);
    }
  }

  // case 2: turning left
  if(state == directionState::TURNING_LEFT) {
    if(dist_front > 0.15 && dist_front_left > 0.22 && dist_front_right > 0.22) {                       // if clear     
      state = directionState::FORWARD;          //go forward
    }
    else{                                       // if not clear 
      send_drive(node, -5.0, 5.0, request, diffDrive);              // turn/continue turning left
    }
  }

  // case 3: turning right
  if(state == directionState::TURNING_RIGHT) {
    if(dist_front > 0.15 && dist_front_left > 0.22 && dist_front_right > 0.22) {                       // if clear
      state = directionState::FORWARD;          // go forward
    }
    else{                                       // if not clear
      send_drive(node, 5.0, -5.0, request, diffDrive);              // turn/continue turning right
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("wanderer");

  auto request = std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
  auto diffDrive = node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive");

  auto laser_output = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", 10, 
    [&] (const sensor_msgs::msg::LaserScan &msg){
      topic_callback(msg, node, request, diffDrive);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
