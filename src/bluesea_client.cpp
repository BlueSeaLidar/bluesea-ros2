#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_srvs/srv/empty.hpp>
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current ROS2 Version is %s",getROS2Version());
  if (argc != 2)
  {                                                                                  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: ros2 run bluesea2  bluesea_node_client stop/start");
    return 1;
  }
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bluesea_node_client"); 
  char cmd[8] = {0};
  strcpy(cmd, argv[1]);
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client;
  if (strcmp(cmd, "start") == 0)
  {
    client = node->create_client<std_srvs::srv::Empty>("start");  // CHANGE
  }
  else if (strcmp(cmd, "stop") == 0)
  { 
    client = node->create_client<std_srvs::srv::Empty>("stop");
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>(); // CHANGE   
  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  //ros2 rolling   20.04
  #ifdef ROS_DISTRO_F
  if(rclcpp::spin_until_future_complete(node, result)==rclcpp::FutureReturnCode::SUCCESS)
  #elif  defined ROS_DISTRO_E
  //ros2 eloquent   18.04 and lower
  if(rclcpp::spin_until_future_complete(node, result)==rclcpp::executor::FutureReturnCode::SUCCESS)
  #endif
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client OK");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call ctrl_service "); // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}
