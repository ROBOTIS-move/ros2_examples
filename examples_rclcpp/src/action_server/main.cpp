#include <memory>
#include <string>
#include <utility>

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_server/lightbulb.hpp"


void print_help()
{
  printf("For action_server(lightBulb) node:\n");
  printf("action_server [-h]\n");
  printf("Options:\n");
  printf("\t-h Help            : Print this help function.\n");
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

  auto node = std::make_shared<robotis::LightBulb>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
