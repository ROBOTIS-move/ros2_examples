#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <rcutils/cmdline_parser.h>

#include "action_server/light.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

void print_help()
{
  printf("For action_server(light) node:\n");
  printf("action_server [-h]\n");
  printf("Options:\n");
  printf("\t-h Help            : Print this help function.\n");
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

  // Load node on shared pointer
  auto node = std::make_shared<robotis::Light>();

  // Create a default single-threaded executor and spin the specified node
  rclcpp::spin(node);

  // The shutdown function causes all nodes and
  // their constituent parts to become invalid and shutdown.
  // It also destroys any global resources created
  // when the initialization function was originally called.
  rclcpp::shutdown();

  return 0;
}

