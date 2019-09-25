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

      explicit Switch();

      bool is_goal_done() const;

      void send_goal();

        
    private:

      rclcpp_action::Client<examples_msgs::action::Led>::SharedPtr action_client_;

      rclcpp::TimerBase::SharedPtr timer_;

      bool goal_done_;

      void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr> future);

      void feedback_callback(
            rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr,
            const std::shared_ptr<const examples_msgs::action::Led::Feedback> feedback
      );

      void result_callback(const rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::WrappedResult & result);

        
    };
}
#endif // EXAMPLES_RCLCPP_CLIENT_SWITCH_HPP_