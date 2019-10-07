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

/* Authors: SungHyeon Oh */

#include <memory>
#include <string>
#include <utility>

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_client/switcher.hpp"


void print_help()
{
  printf("For server_client(switcher) node:\n");
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
