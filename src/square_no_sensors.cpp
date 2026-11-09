#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wanderer");

  auto request = std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
  auto diffDrive = node->create_client<create_fundamentals::srv::DiffDrive>("diff_drive");
  while(!diffDrive->wait_for_service(std::chrono::seconds(1))){
	RCLCPP_INFO(node->get_logger(), "waiting for service..");
  }    
  for (int i = 0; i < 4; i++) {
    
    request->left = 10.42;
    request->right = 10.42;
    diffDrive->async_send_request(request);

    rclcpp::sleep_for(std::chrono::seconds(3));

    request->left = 0;
    request->right = 0;
    diffDrive->async_send_request(request);

    rclcpp::sleep_for(std::chrono::seconds(1));

    request->left = 3.17;
    request->right = -3.17;
    diffDrive->async_send_request(request);

    rclcpp::sleep_for(std::chrono::seconds(2));

    request->left = 0;
    request->right = 0;
    diffDrive->async_send_request(request);

    rclcpp::sleep_for(std::chrono::seconds(1));

  }

  request->left = 0;
  request->right = 0;
  diffDrive->async_send_request(request);
  

  
  rclcpp::shutdown();
  return 0;
}
