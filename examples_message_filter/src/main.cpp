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

/* Authors: Darby Lim */

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
// Load user-defined header
#include "examples_message_filter/publisher.hpp"
#include "examples_message_filter/message_filter.hpp"


void print_help()
{
  printf("For message_filter node:\n");
  printf("examples_message_filter [-h]\n");
  // TODO: Added arguments
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

  // Specify rclcp executor
  // executor makes node to sequential execution
  // https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/executors/single_threaded_executor.cpp
  rclcpp::executors::MultiThreadedExecutor executor;

  // Specify lifecycle node
  using namespace std::chrono_literals;
  auto pub1 = std::make_shared<robotis::Counter>("first", "first", 73ms);
  auto pub2 = std::make_shared<robotis::Counter>("second", "second", 37ms);
  auto filter = std::make_shared<robotis::MsgFilters>(10, "first", "second");

  // Added node to executor
  executor.add_node(pub1);
  executor.add_node(pub2);
  executor.add_node(filter);

  // Spin a node
  executor.spin();

  // The shutdown function causes all nodes and
  // their constituent parts to become invalid and shutdown.
  // It also destroys any global resources created
  // when the initialization function was originally called.
  rclcpp::shutdown();

  return 0;
}
