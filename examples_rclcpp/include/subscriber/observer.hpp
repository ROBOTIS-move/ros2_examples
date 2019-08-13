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

#ifndef EXAMPLES_RCLCPP_SUBSCRIBER_OBSERVER_HPP_
#define EXAMPLES_RCLCPP_SUBSCRIBER_OBSERVER_HPP_

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
class Observer : public rclcpp::Node
{
 public:
  // 'explicit' makes block data conversion
  explicit Observer(const int32_t & qos_profile);

 private:
  // Initialize parameters
  void init_parameters();

  // Subscribe parameter events
  void parameter_event_callback();

   // Return QoS Class by setting qos_profile
  rclcpp::QoS get_qos(const int32_t & qos_profile);

  // After C++11, member variables can be initialized when it was declared at the same time

  // Declare blind flag
  bool blind_ = false;

  // Declare offset variable to add subscribed data
  int64_t offset_ = 0;

  // Declare comment string
  std::string comment_ = "Hello";

  // Declare subscriber as shared pointer
  rclcpp::Subscription<examples_msgs::msg::Count>::SharedPtr sub_;

  // Declare subscriber to get parameter event
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  // Declare parameter client to handle API(get, set, ...)
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
} // robotis
#endif // EXAMPLES_RCLCPP_SUBSCRIBER_OBSERVER_HPP_
