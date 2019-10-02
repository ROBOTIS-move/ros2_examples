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

    bool is_goal_done() const;

    int32_t user_custom_value_ = 5;

   private:
    void print_message_info();

    rclcpp::TimerBase::SharedPtr timer_;

    bool goal_done_ = false;

    rclcpp_action::Client<examples_msgs::action::Led>::SharedPtr action_client_;

  };
}
#endif // EXAMPLES_RCLCPP_CLIENT_SWITCH_HPP_
