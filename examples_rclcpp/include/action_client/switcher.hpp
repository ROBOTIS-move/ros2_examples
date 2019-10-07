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

#ifndef EXAMPLES_RCLCPP_ACTION_CLIENT_SWITCHER_HPP_
#define EXAMPLES_RCLCPP_ACTION_CLIENT_SWITCHER_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "examples_msgs/action/led.hpp"


namespace robotis
{
class Switcher : public rclcpp::Node
{
 public:
  using Led = examples_msgs::action::Led;

  explicit Switcher(const int32_t input_value);
  virtual ~Switcher(){};

  void send_goal();

  bool is_goal_done() const;

  int32_t user_custom_value_ = 5;

 private:
  void print_message_info();

  void goal_response_callback(
    std::shared_future<rclcpp_action::ClientGoalHandle<Led>::SharedPtr> future);

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<Led>::SharedPtr,
    const std::shared_ptr<const Led::Feedback> feedback);

  void result_callback(
    const rclcpp_action::ClientGoalHandle<Led>::WrappedResult & output);

  bool goal_done_ = false;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::Client<Led>::SharedPtr action_client_;
};
}
#endif // EXAMPLES_RCLCPP_CLIENT_SWITCH_HPP_
