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

#include "examples_lifecycle/robot.hpp"

using namespace robotis;

// https://en.cppreference.com/w/cpp/symbol_index/chrono_literals
using namespace std::chrono_literals;

Robot::Robot(const bool & auto_activate)
: rclcpp_lifecycle::LifecycleNode("robot", rclcpp::NodeOptions().use_intra_process_comms(true)),
  robot_name_("R2D2"),
  robot_state_("stop")
{
  RCLCPP_INFO(this->get_logger(), "--------------------Robot() is called");
  RCLCPP_INFO(this->get_logger(), "--------------------Please set configure");

  if (auto_activate)
  {
    client_change_state_ =
      this->create_client<lifecycle_msgs::srv::ChangeState>("robot/change_state");

    if (!client_change_state_->wait_for_service(1s)) {
      RCLCPP_ERROR(
        get_logger(),
        "Service %s is not available.",
        client_change_state_->get_service_name());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "--------------------Auto Activate!");

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

    client_change_state_->async_send_request(request);

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

    client_change_state_->async_send_request(request);
  }
}

LifecycleReturn Robot::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "--------------------on_configure() is called");

  this->get_parameters();
  this->plugin_robot();

  return LifecycleReturn::SUCCESS;
}

LifecycleReturn Robot::on_activate(const rclcpp_lifecycle::State &)
{
  pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "--------------------on_activate() is called");

  this->run();

  return LifecycleReturn::SUCCESS;
}

LifecycleReturn Robot::on_deactivate(const rclcpp_lifecycle::State &)
{
  pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "--------------------on_deactivate() is called");

  this->stop();

  return LifecycleReturn::SUCCESS;
}

LifecycleReturn Robot::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "--------------------on_cleanup() is called");

  this->unplug_robot();

  return LifecycleReturn::SUCCESS;
}

LifecycleReturn Robot::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "--------------------on_shutdown() is called");

  this->unplug_robot();

  return LifecycleReturn::SUCCESS;
}

void Robot::get_parameters()
{
  // Declare parameters that may be set on this node
  // https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#declaring-a-parameter-with-a-parameterdescriptor
  this->declare_parameter("robot_name", "R2D2");

  robot_name_ = this->get_parameter("robot_name").get_value<std::string>();

  RCLCPP_INFO(this->get_logger(), "My name is %s", robot_name_.c_str());
}

void Robot::plugin_robot()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pub_ = this->create_publisher<std_msgs::msg::String>("state", qos);

  // Rambda function (https://en.cppreference.com/w/cpp/language/lambda)
  timer_ = this->create_wall_timer(
    500ms,
    [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::String>();
        msg_->data = robot_state_;

        RCLCPP_INFO(
          this->get_logger(),
          "[%s] State: '%s'",
          robot_name_.c_str(), robot_state_.c_str());

        // 'move' let tranfer unique_ptr's ownership
        pub_->publish(std::move(msg_));
      }
    );

    RCLCPP_INFO(this->get_logger(), "%s is plugged in", robot_name_.c_str());
}

void Robot::unplug_robot()
{
  timer_.reset();
  pub_.reset();

  RCLCPP_INFO(this->get_logger(), "%s is unplugged", robot_name_.c_str());
}

void Robot::run()
{
  robot_state_ = "run";
}

void Robot::stop()
{
  robot_state_ = "stop";
}


