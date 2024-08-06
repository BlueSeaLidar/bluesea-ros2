#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "base/srv/control.hpp"
int main(int argc, char **argv)
{
  if (argc != 3&&argc != 4)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"usage: ros2 run bluesea2  bluesea2_client scan start                          arg1 is topic   arg2 is action(start/stop)");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"usage: ros2 run bluesea2  bluesea2_client scan switchZone  0                  arg1 is topic   arg2 is action(switchZone)  arg3 is select zone id");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"usage: ros2 run bluesea2  bluesea2_client scan rpm  600                       arg1 is topic   arg2 is action(set rpm)  arg3 is rpm value");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"usage: ros2 run bluesea2  bluesea2_client heart check  1                      arg1 is service name   arg2 is action(check)  arg3 is print / not print");
    return 1;
  }
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bluesea_node_client"); 
  char serviceName[64] = {0};
  sprintf(serviceName,"/%s_motor",argv[1]);

  rclcpp::Client<base::srv::Control>::SharedPtr client = node->create_client<base::srv::Control>(serviceName);  // CHANGE
  //Retain four universal parameters
  auto request = std::make_shared<base::srv::Control::Request>(); 
  request->topic=argv[1];
  request->func=argv[2];
  if(strcmp(argv[2],"start")==0||strcmp(argv[2],"stop")==0)
  {
  }
  else if(strcmp(argv[2],"switchZone")==0)
  {
    request->flag=atoi(argv[3]);
  }
  else if(strcmp(argv[2],"rpm")==0)
  {
    request->params=argv[3];
    
  }
  else if(strcmp(argv[2],"check")==0)
  {
    request->flag=atoi(argv[3]);
  }
 
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

