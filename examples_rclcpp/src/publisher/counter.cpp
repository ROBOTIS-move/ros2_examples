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

#include "publisher/counter.hpp"

using namespace robotis;

// https://en.cppreference.com/w/cpp/symbol_index/chrono_literals
using namespace std::chrono_literals;

Counter::Counter(const std::string & comment, const int32_t & qos_profile)
: Node("counter")
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");

  // Declare parameters that may be set on this node
  // https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#declaring-a-parameter-with-a-parameterdescriptor
  this->declare_parameter(
    "comment",
    rclcpp::ParameterValue("No comments"),
    rcl_interfaces::msg::ParameterDescriptor());

  // Get parameter from yaml
  this->get_parameter("comment");

  auto qos = get_qos(qos_profile);
  pub_ = this->create_publisher<examples_msgs::msg::Count>("count", qos);

  // Rambda function (https://en.cppreference.com/w/cpp/language/lambda)
  timer_ = this->create_wall_timer(
    1s,
    [this, comment]() -> void
      {
        msg_ = std::make_unique<examples_msgs::msg::Count>();
        msg_->header.stamp = this->now();
        msg_->count = count_++;
        RCLCPP_INFO(this->get_logger(), "[%s] Counting: '%d'", comment.c_str(), msg_->count);

        // 'move' let tranfer unique_ptr's ownership
        pub_->publish(std::move(msg_));
      }
    );
}

rclcpp::QoS Counter::get_qos(const int32_t & qos_profile)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  if (qos_profile == 0)
  {
    qos = qos;
  }
  else if (qos_profile == 1)
  {
    qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  }
  else if (qos_profile == 2)
  {
    qos = rclcpp::QoS(rclcpp::ParametersQoS());
  }
  else if (qos_profile == 3)
  {
    qos = rclcpp::QoS(rclcpp::ServicesQoS());
  }
  else if (qos_profile == 4)
  {
    qos = rclcpp::QoS(rclcpp::ParameterEventsQoS());
  }
  else if (qos_profile == 5)
  {
    qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  }
  else
  {
    qos = qos;
  }

  return qos;
}
