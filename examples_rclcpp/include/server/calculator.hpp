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

#ifndef EXAMPLES_RCLCPP_SERVER_CALCULATOR_HPP_
#define EXAMPLES_RCLCPP_SERVER_CALCULATOR_HPP_

// Load smart pointer
#include <memory>
// Load std::string
#include <string>
// Load move, forward, tuple and etc (https://en.cppreference.com/w/cpp/header/utility)
#include <utility>

// Load user-defined msgs
#include <examples_msgs/srv/calculation.hpp>
// Load C++ ROS2 Client Library (http://docs.ros2.org/dashing/api/rclcpp/index.html)
#include <rclcpp/rclcpp.hpp>



namespace robotis
{
class Calculator : public rclcpp::Node
{
 public:
  // 'explicit' makes block data conversion
  explicit Calculator();

 private:
  // Print help message to user
  void message_info();

  // Calculate `a` and `b` by arithmetic_operator
  int32_t calculation(
    const int32_t & a,
    const int32_t & b,
    const std::string & arithmetic_operator);

  // Declare service server
  rclcpp::Service<examples_msgs::srv::Calculation>::SharedPtr srv_;
};
} // robotis
#endif // EXAMPLES_RCLCPP_SERVER_CALCULATOR_HPP_
