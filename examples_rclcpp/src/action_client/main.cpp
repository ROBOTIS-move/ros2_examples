#include <inttypes.h>
#include <memory>
#include <string>
#include <utility>

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_client/switcher.hpp"


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
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
  {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  char * cmd_line_interface_option = rcutils_cli_get_option(argv, argv+argc, "-n");

  int input_value = 5;

  if (nullptr != cmd_line_interface_option)
  {
    input_value = atoi(cmd_line_interface_option);
  }

  auto node = std::make_shared<robotis::Switcher>(input_value);

  while (!node->is_goal_done())
  {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return 0;
}
