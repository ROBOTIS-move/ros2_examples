#ifndef EXAMPLES_RCLCPP_ACTION_SERVER_LIGHTOR_HPP_
#define EXAMPLES_RCLCPP_ACTION_SERVER_LIGHTOR_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "examples_msgs/action/led.hpp"


namespace robotis
{
class Lightor : public rclcpp::Node
{
 public:
  explicit Lightor();
  virtual ~Lightor(){};

 private:
  void print_message_info();

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle);

  rclcpp_action::Server<examples_msgs::action::Led>::SharedPtr action_server_;
};
}
#endif // EXAMPLES_RCLCPP_SERVER_CALCULATOR_HPP_
