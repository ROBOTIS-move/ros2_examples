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

#ifndef EXAMPLES_RCLCPP_PUBLISHER_COUNTER_HPP_
#define EXAMPLES_RCLCPP_PUBLISHER_COUNTER_HPP_

// Load smart pointer
#include <memory>
// Load std::string
#include <string>
// Load move, forward, tuple and etc (https://en.cppreference.com/w/cpp/header/utility)
#include <utility>

// Load user-defined msgs
#include <examples_msgs/msg/count.hpp>
// Load C++ ROS2 Client Library (http://docs.ros2.org/dashing/api/rclcpp/index.html)
#include <rclcpp/rclcpp.hpp>

namespace robotis
{
class Counter : public rclcpp::Node
{
 public:
  // 'explicit' makes block data conversion
  explicit Counter(const std::string & comment, const int32_t & qos_profile);

 private:
  // Return QoS Class by setting qos_profile
  rclcpp::QoS get_qos(const int32_t & qos_profile);

  // After C++11, member variables can be initialized when it was declared at the same time
  // 'size_t' can be store the maximum size of a theoretically possible object of any type

  // Declare count variable
  size_t count_ = 1;

  // Declare topic message as unique pointer
  std::unique_ptr<examples_msgs::msg::Count> msg_;

  // Declare publisher as shared pointer
  rclcpp::Publisher<examples_msgs::msg::Count>::SharedPtr pub_;

  // Declare timer to publish data periodically
  rclcpp::TimerBase::SharedPtr timer_;
};
} // robotis
#endif // EXAMPLES_RCLCPP_PUBLISHER_COUNTER_HPP_
