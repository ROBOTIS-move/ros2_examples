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

#ifndef EXAMPLES_MESSAGE_FILTER_PUBLISHER_HPP_
#define EXAMPLES_MESSAGE_FILTER_PUBLISHER_HPP_

// Load chrono
#include <chrono>
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
  explicit Counter(
    const std::string & node_name,
    const std::string & topic_name,
    const std::chrono::milliseconds timeout)
  : Node(node_name)
  {
    RCLCPP_INFO(this->get_logger(), "%s node init", node_name.c_str());

    pub_ = this->create_publisher<examples_msgs::msg::Count>(topic_name, rclcpp::QoS(rclcpp::KeepLast(10)));

    // Rambda function (https://en.cppreference.com/w/cpp/language/lambda)
    timer_ = this->create_wall_timer(
      timeout,
      [this]() -> void
        {
          msg_ = std::make_unique<examples_msgs::msg::Count>();
          msg_->header.stamp = this->now();
          msg_->count = count_++;
          RCLCPP_INFO(this->get_logger(), "Counting: '%d'", msg_->count);

          // 'move' let tranfer unique_ptr's ownership
          pub_->publish(std::move(msg_));
        }
      );
  }

 private:
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
#endif // EXAMPLES_MESSAGE_FILTER_PUBLISHER_HPP_
