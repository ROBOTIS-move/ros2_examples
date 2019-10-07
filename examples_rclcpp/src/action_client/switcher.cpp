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

#include "action_client/switcher.hpp"

using namespace robotis;
using namespace std::chrono_literals;

Switcher::Switcher(const int32_t input_value)
: Node("switch"),
  goal_done_(false)
{
  print_message_info();

  this->action_client_ = rclcpp_action::create_client<Led>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "Led_on");

  this->user_custom_value_ = input_value;

  this->timer_ = this->create_wall_timer(
    0.5s,
    std::bind(&Switcher::send_goal, this));
}

bool Switcher::is_goal_done() const
{
  return goal_done_;
}

void Switcher::send_goal()
{
  using namespace std::placeholders;

  this->timer_->cancel();

  this->goal_done_ = false;

  if (!this->action_client_)
  {
    if (!action_client_)
    {
      RCLCPP_INFO(this->get_logger(), "Action client not initialized");
    }

    if (!this->action_client_->wait_for_action_server(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");

      this->goal_done_ = true;

      return;
    }
  }

  auto goal = Led::Goal();

  goal.numbers = Switcher::user_custom_value_;

  auto send_goal_options = rclcpp_action::Client<Led>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&Switcher::goal_response_callback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&Switcher::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&Switcher::result_callback, this, _1);

  auto goal_handle_future = this->action_client_->async_send_goal(goal, send_goal_options);
}

void Switcher::goal_response_callback(
  std::shared_future<rclcpp_action::ClientGoalHandle<Led>::SharedPtr> future)
{
  auto goal_handle = future.get();

  if (!goal_handle)
  {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by action server");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Goal accepted by action server, waiting for result");
  }
}

void Switcher::feedback_callback(
  rclcpp_action::ClientGoalHandle<Led>::SharedPtr,
  const std::shared_ptr<const Led::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "Next result : %s" , feedback->process.back().c_str());
}

void Switcher::result_callback(
  const rclcpp_action::ClientGoalHandle<Led>::WrappedResult & output)
{
  goal_done_ = true;

  switch (output.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      RCLCPP_INFO(get_logger(), "Goal was succeeded");
      break;
    }
    case rclcpp_action::ResultCode::ABORTED:
    {
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    }
    case rclcpp_action::ResultCode::CANCELED:
    {
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      return;
    }
    default:
    {
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "Result received");

  for (auto number : output.result->result)
  {
    RCLCPP_INFO(get_logger(), "%s", number.c_str());
  }
}

void Switcher::print_message_info()
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");
  RCLCPP_INFO(this->get_logger(), "switcher calls lightbulb!");
}
