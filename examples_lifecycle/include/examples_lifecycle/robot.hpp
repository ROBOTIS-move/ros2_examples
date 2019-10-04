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

#ifndef EXAMPLES_LIFECYCLE_ROBOT_HPP_
#define EXAMPLES_LIFECYCLE_ROBOT_HPP_

// Load smart pointer
#include <memory>
// Load std::string
#include <string>
// Load move, forward, tuple and etc (https://en.cppreference.com/w/cpp/header/utility)
#include <utility>
// Load change_state service in lifecycle_msgs
#include <lifecycle_msgs/srv/change_state.hpp>
// Load RCLCPP Lifecycle Node Library (https://github.com/ros2/rclcpp/tree/master/rclcpp_lifecycle)
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// Load string message in std_msgs
#include <std_msgs/msg/string.hpp>


using LifecycleReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace robotis
{
class Robot : public rclcpp_lifecycle::LifecycleNode
{
 public:
  // 'explicit' makes block data conversion
  explicit Robot(const bool & auto_activate);

  // Specify overriding functions
  LifecycleReturn on_configure(const rclcpp_lifecycle::State &) override;
  LifecycleReturn on_activate(const rclcpp_lifecycle::State &) override;
  LifecycleReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  LifecycleReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  LifecycleReturn on_shutdown(const rclcpp_lifecycle::State &) override;

 private:
  // Get all parameters when you set in launch file or yaml
  void get_parameters();

  // Initialization robot
  void plugin_robot();

  // Shutdown robot
  void unplug_robot();

  // Declare status of robot
  void run();

  // Declare status of robot
  void stop();

  // Declare robot name. This variable will be set by parameters
  std::string robot_name_;

  // Declare robot state. This variable will be changed by lifecycle state
  std::string robot_state_;

  // Declare topic message as unique pointer
  std::unique_ptr<std_msgs::msg::String> msg_;

  // Declare publisher as shared pointer
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

  // Declare timer to publish data periodically
  rclcpp::TimerBase::SharedPtr timer_;

  // Declare client to change state
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};
} // robotis
#endif // EXAMPLES_LIFECYCLE_ROBOT_HPP_
