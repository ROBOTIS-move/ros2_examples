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
  explicit Switcher(const int32_t input_value);
  virtual ~Switcher(){};

  void send_goal();

  bool is_goal_done() const;

  int32_t user_custom_value_ = 5;

 private:
  void print_message_info();

  void goal_response_callback(
    std::shared_future<rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr> future);

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr,
    const std::shared_ptr<const examples_msgs::action::Led::Feedback> feedback);

  void result_callback(
    const rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::WrappedResult & result);

  bool goal_done_ = false;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::Client<examples_msgs::action::Led>::SharedPtr action_client_;
};
}
#endif // EXAMPLES_RCLCPP_CLIENT_SWITCH_HPP_
