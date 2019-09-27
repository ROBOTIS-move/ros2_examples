#ifndef EXAMPLES_RCLCPP_ACTION_CLIENT_SWITCH_HPP_
#define EXAMPLES_RCLCPP_ACTION_CLIENT_SWITCH_HPP_


#include <memory>

#include <string>

#include <utility>

#include <examples_msgs/action/led.hpp>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

namespace robotis
{
    class Switch : public rclcpp::Node
    {
    public:
      // 'explicit' makes block data conversion
      explicit Switch(const int32_t input_value);

      bool is_goal_done() const;
      
      int32_t Custom_value;
          
    private:
      // Print help message to user
      void message_info();

      rclcpp::TimerBase::SharedPtr timer_;

      bool goal_done_;

      // Declare action client
      rclcpp_action::Client<examples_msgs::action::Led>::SharedPtr action_client_;
      
    };
}
#endif // EXAMPLES_RCLCPP_CLIENT_SWITCH_HPP_