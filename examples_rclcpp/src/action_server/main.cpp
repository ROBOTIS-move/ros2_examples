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

/* Authors: OhSungHyeon */

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
