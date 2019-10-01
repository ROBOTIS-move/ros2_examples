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

#include "action_server/light.hpp"

using namespace robotis;

Light::Light()
: Node("light")
{
  message_info();
  using namespace std::placeholders;

  auto handle_goal =
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const examples_msgs::action::Led::Goal> goal) -> rclcpp_action::GoalResponse
      {
        RCLCPP_INFO(get_logger(), "Received goal request %d", goal->numbers);
        (void)uuid;

        if(goal->numbers>6){
         return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

  auto handle_cancel =
    [this](
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle) -> rclcpp_action::CancelResponse
      {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      };
  auto handle_accepted =
    [this](
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle) -> void
      {
        using namespace std::placeholders;
        std::thread{std::bind(&Light::execute,this,_1),goal_handle}.detach();
      };

  action_server_=rclcpp_action::create_server<examples_msgs::action::Led>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "Led_on",
    handle_goal,
    handle_cancel,
    handle_accepted);
}

void Light::message_info()
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");
  RCLCPP_INFO(
  this->get_logger(),
  "ros2 action_server call /Led_on examples_msgs/action/Led \"{numbers : 5}\"");
}

void Light::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();

  auto feedback=std::make_shared<examples_msgs::action::Led::Feedback>();
  auto & process = feedback->process;

  auto result=std::make_shared<examples_msgs::action::Led::Result>();

  std::string lantern="[ ][ ][ ][ ][ ]";

  for(int i = 0; (i<goal->numbers) && rclcpp::ok(); ++i)
  {
    if(goal_handle->is_canceling())
    {
      result->result=process;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal Canceled");
      return;
    }

    lantern[3 * i + 1]='O';
    process.push_back(lantern);

    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Publish Feedback");

    loop_rate.sleep();
  }

  if(rclcpp::ok())
  {
    result->result=process;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Achieve the goal");
  }

}
