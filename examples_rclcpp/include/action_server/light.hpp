#ifndef EXAMPLES_RCLCPP_ACTION_SERVER_LIGHT_HPP_
#define EXAMPLES_RCLCPP_ACTION_SERVER_LIGHT_HPP_


#include <memory>

#include <string>

#include <utility>

#include <examples_msgs/action/led.hpp>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"



namespace robotis
{
    class Light : public rclcpp::Node
    {
    public:   
      // 'explicit' makes block data conversion
      explicit Light();

    private:
      void message_info();
      
      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const examples_msgs::action::Led::Goal> goal
        );
      


      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle
      );


      void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle);


      void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle);


      rclcpp_action::Server<examples_msgs::action::Led>::SharedPtr action_server_;

      //const rclcpp::NodeOptions & options=rclcpp::NodeOptions()
    };
     
}
#endif // EXAMPLES_RCLCPP_SERVER_CALCULATOR_HPP_