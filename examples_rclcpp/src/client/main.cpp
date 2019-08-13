/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Pyo */

// Load setvbuf, printf
#include <cstdio>
// Load smart pointer
#include <memory>
// Load std::string
#include <string>
// Load move, forward, tuple and etc (https://en.cppreference.com/w/cpp/header/utility)
#include <utility>

// Load C++ ROS2 Client Library (http://docs.ros2.org/dashing/api/rclcpp/index.html)
#include <rclcpp/rclcpp.hpp>
// Load ROS2 C utilities data structures (https://github.com/ros2/rcutils)
#include <rcutils/cmdline_parser.h>

#include "client/requestor.hpp"

void print_help()
{
  printf("For server(requestor) node:\n");
  printf("server [-h] [-a number] [-b number] [-o operator]\n");
  printf("Options:\n");
  printf("\t-h Help            : Print this help function.\n");
  printf("Defaults to calculate.\n");
  printf("\t-a ${number}       : Specify the number. Defaults to 1.\n");
  printf("\t-b ${number}       : Specify the number. Defaults to 2.\n");
  printf("\t-o ${operator}     : Specify the arithmetic operator(plus, minus, multiply, division)");
  printf("\tDefaults to plus.\n");
}

int main(int argc, char * argv[])
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

  char * cli_option[3];

  // Get data from specified argument
  cli_option[0] = rcutils_cli_get_option(argv, argv + argc, "-a");
  cli_option[1] = rcutils_cli_get_option(argv, argv + argc, "-b");
  cli_option[2] = rcutils_cli_get_option(argv, argv + argc, "-o");

  int a = 1;
  if (nullptr != cli_option[0])
  {
    a = atoi(cli_option[0]);
  }

  int b = 2;
  if (nullptr != cli_option[1])
  {
    b = atoi(cli_option[1]);
  }

  auto arithmetic_operator = std::string("plus");
  if (nullptr != cli_option[2])
  {
    arithmetic_operator = std::string(cli_option[2]);
  }

  // Load node on shared pointer
  auto node = std::make_shared<robotis::Requestor>(a, b, arithmetic_operator);

  // Create a default single-threaded executor and spin the specified node
  rclcpp::spin(node);

  // The shutdown function causes all nodes and
  // their constituent parts to become invalid and shutdown.
  // It also destroys any global resources created
  // when the initialization function was originally called.
  rclcpp::shutdown();

  return 0;
}
