#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <rcutils/cmdline_parser.h>

#include "action_client/switch.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include <inttypes.h>

void print_help()
{
  printf("For server_client(switch) node:\n");
  printf("action_client [-h] [-n number]\n");
  printf("Options:\n");
  printf("\t-h Help            : Print this help function.\n");
  printf("Default to activate lights.\n");
  printf("\t-n ${number}       : Specify the number. Default to 5.\n");
}

int main(int argc, char *argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Find specified argument option
  if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
  {
    print_help();
    return 0;
  }

  // This function initializes any global resources needed by the middleware and the client library,
  // as well as doing client library related command line argument parsing.
  rclcpp::init(argc, argv);
  
  // Get data from specified argument
  char * cli_option = rcutils_cli_get_option(argv, argv+argc, "-n");

  int input_value = 5;
  if(nullptr != cli_option)
  {
     input_value=atoi(cli_option);
  }

  // Load node on shared pointer
  auto node = std::make_shared<robotis::Switch>(input_value);

  while(!node->is_goal_done())
  {
    rclcpp::spin_some(node);
  }

  // The shutdown function causes all nodes and
  // their constituent parts to become invalid and shutdown.
  // It also destroys any global resources created
  // when the initialization function was originally called.
  rclcpp::shutdown();


  return 0;

}