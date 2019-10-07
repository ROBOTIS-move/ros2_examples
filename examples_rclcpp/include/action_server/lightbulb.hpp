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

/* Authors: OhSungHyeon */

#ifndef EXAMPLES_RCLCPP_ACTION_SERVER_LIGHTBULB_HPP_
#define EXAMPLES_RCLCPP_ACTION_SERVER_LIGHTBULB_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "examples_msgs/action/led.hpp"


namespace robotis
{
class LightBulb : public rclcpp::Node
{
 public:
  explicit LightBulb();
  virtual ~LightBulb(){};

 private:
  void print_message_info();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const examples_msgs::action::Led::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle);

  rclcpp_action::Server<examples_msgs::action::Led>::SharedPtr action_server_;

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle);
};
}
#endif // EXAMPLES_RCLCPP_SERVER_CALCULATOR_HPP_
