#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <rcutils/cmdline_parser.h>

#include "action_client/switch.hpp"

#include "rclcpp_action/rclcpp_action.hpp"


#include <inttypes.h>


int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  
  rclcpp::init(argc, argv);

 
  auto node = std::make_shared<robotis::Switch>();

  while(!node->is_goal_done())
  {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();


  return 0;

}