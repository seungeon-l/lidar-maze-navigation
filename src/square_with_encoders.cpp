#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include <cmath>

const float r = 0.032;
const float b = 0.257;
const float encoder_correction_factor = 0.93;

float encoder_left = 0.0;
float encoder_right = 0.0;


void send_drive(rclcpp::Node::SharedPtr node, float left, float right) {
  auto request = std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
  auto diffDrive = node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive"); 
  request->left = left;
  request->right = right;
  diffDrive->async_send_request(request);
}

// always input positive values
float calculate_speed(float dist, float dist_desired, float k) {
  return k * (dist_desired - dist);
}


void sensor_callback(const create_fundamentals::msg::SensorPacket &msg, rclcpp::Node::SharedPtr node) {
    encoder_left = msg.encoder_left;
    encoder_right = msg.encoder_right;

 
     
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wanderer");

  auto request = std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
  auto diffDrive = node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive");

  auto sensor_output = node->create_subscription<create_fundamentals::msg::SensorPacket>(
    "sensor_packet", 10, 
    [&] (const create_fundamentals::msg::SensorPacket &msg){
      sensor_callback(msg, node);
    });

  auto reset = std::make_shared<create_fundamentals::srv::ResetEncoders::Request>();
  auto resetEnc = node->create_client<create_fundamentals::srv::ResetEncoders>("reset_encoders");

  rclcpp::Rate rate(100);
 
  for (int i = 0; i < 4; i++) {
    resetEnc->async_send_request(reset);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    
    
    // 1. Moving straight
    float speed = 0.0; 
    float dist_desired = 1.0;
    float k = 15;
    while(rclcpp::ok()) {

      double dist = r * encoder_right;
      speed = calculate_speed(dist, dist_desired, k);

      request->left = fmax(speed, 2);
      request->right = fmax(speed, 2);
      diffDrive->async_send_request(request);

      // RCLCPP_INFO(node->get_logger(), "encoder_left: %f", encoder_left);
      // RCLCPP_INFO(node->get_logger(), "encoder_right: %f", encoder_right);
      // RCLCPP_INFO(node->get_logger(), "dist: %f", dist);

      if (dist_desired - dist <= 0.005) {
          break;
      }

      rclcpp::spin_some(node);
      rate.sleep();
    }

    resetEnc->async_send_request(reset);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // Halt

    request->left = 0.0;
    request->right = 0.0;
    diffDrive->async_send_request(request);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    resetEnc->async_send_request(reset);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // RCLCPP_INFO(node->get_logger(), "enc. left: %f", encoder_left);
    // RCLCPP_INFO(node->get_logger(), "enc. right: %f", encoder_right);

    // 2. Make a turn - Approach 1: use angle
    // float speed_turn = 0.0;
    // float angle_desired = M_PI_2;
    // float k_angle = 0.1;

    // while(rclcpp::ok()) {

    //   double angle = (encoder_left - encoder_right) * r / b;
    //   double speed_angle = calculate_speed(angle, angle_desired, k_angle);

    //   request->left = fmax(speed, 1);
    //   request->right = -fmax(speed, 1);
    //   diffDrive->async_send_request(request);

    //   RCLCPP_INFO(node->get_logger(), "angle: %f", angle);

    //   if (abs(angle - angle_desired) <= 0.01) {
    //       break;
    //   }

    //   rclcpp::spin_some(node);
    //   rate.sleep();
    // }

    // 2. Make a turn - Approach 2: use distance
    RCLCPP_INFO(node->get_logger(), "\n\nturn %i", i); 

    float speed_turn = 0.0; 
    float dist_desired_turn = M_PI_4 * b;
    float k_turn = 22;

    resetEnc->async_send_request(reset);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    while(rclcpp::ok()) {

      double dist_turn = r * (abs(encoder_right));
      speed_turn = calculate_speed(dist_turn, dist_desired_turn, k_turn);

      request->left = fmax(speed_turn, 2);
      request->right = -fmax(speed_turn, 2);
      diffDrive->async_send_request(request);

      double angle = (abs(encoder_right)) * r / b;

      RCLCPP_INFO(node->get_logger(), "encoder_left: %f", encoder_left);
      RCLCPP_INFO(node->get_logger(), "encoder_right: %f", encoder_right);
      RCLCPP_INFO(node->get_logger(), "difference: %f", abs(encoder_right) - abs(encoder_left));
      RCLCPP_INFO(node->get_logger(), "angle: %f", angle);
      RCLCPP_INFO(node->get_logger(), "----------------------\n");

      if (abs(dist_turn - dist_desired_turn) <= 0.003) {
          break;
      }

      rclcpp::spin_some(node);
      rate.sleep();
    }

    request->left = 0.0;
    request->right = 0.0;
    diffDrive->async_send_request(request);

    resetEnc->async_send_request(reset);

    rclcpp::sleep_for(std::chrono::milliseconds(200));

  }
  
  rclcpp::shutdown();
  return 0;
}
