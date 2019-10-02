#ifndef EXAMPLES_RCLCPP_ACTION_SERVER_LIGHTOR_HPP_
#define EXAMPLES_RCLCPP_ACTION_SERVER_LIGHTOR_HPP_

#include <memory>
#include <string>
#include <utility>

#include <examples_msgs/action/led.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace robotis
{
    class Lightor : public rclcpp::Node
    {
    public:
      // 'explicit' makes block data conversion
      explicit Lightor();

    private:
      // Print help message to user
      void print_message_info();

      void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle);

      // Declare action server
      rclcpp_action::Server<examples_msgs::action::Led>::SharedPtr action_server_;

    };

}
#endif // EXAMPLES_RCLCPP_SERVER_CALCULATOR_HPP_