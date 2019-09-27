#include "action_client/switch.hpp"
#include <inttypes.h>

using namespace robotis;

using namespace std::chrono_literals;

Switch::Switch(const int32_t input_value)
: Node("switch"), goal_done_(false)
{
    message_info();

    this->action_client_=rclcpp_action::create_client<examples_msgs::action::Led>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "Led_on");

        this->Custom_value=input_value;

    timer_=this->create_wall_timer(
        0.5s,
        [this]() ->void
        {
            using namespace std::placeholders;

            timer_->cancel();

            goal_done_=false;

            if(!action_client_){
               RCLCPP_INFO(get_logger(), "Action client not initialized");
            }

            if(!action_client_->wait_for_action_server(10s)){
                RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
                goal_done_=true;
                return;
            }

            auto goal=examples_msgs::action::Led::Goal();
            goal.numbers=Switch::Custom_value;

            RCLCPP_INFO(get_logger(), "Sending goal");

            auto send_goal_options=rclcpp_action::Client<examples_msgs::action::Led>::SendGoalOptions();
            //send_goal_options.goal_response_callback = std::bind(&Switch::goal_response_callback, this, _1);
            send_goal_options.goal_response_callback = 
            [this](
                std::shared_future<rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr>future) -> void
                {
                    auto goal_handle=future.get();
                    if(!goal_handle)
                    {
                        RCLCPP_ERROR(get_logger(), "Goal was rejected by action server");
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(), "Goal accepted by action server, waiting for result");    
                    }

                };
            // send_goal_options.feedback_callback = std::bind(&Switch::feedback_callback, this, _1, _2);
            send_goal_options.feedback_callback =
            [this](
                rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr,
                const std::shared_ptr<const examples_msgs::action::Led::Feedback> feedback) -> void
                {
                    RCLCPP_INFO(get_logger(),"Next result in sequence received: %s" ,feedback->process.back().c_str());
                };
            
            // send_goal_options.result_callback=std::bind(&Switch::result_callback,this,_1);
            send_goal_options.result_callback =
            [this](
                const rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::WrappedResult & result) -> void
                {
                    goal_done_=true;
                    switch(result.code){
                       case rclcpp_action::ResultCode::SUCCEEDED:
                          break;
                       case rclcpp_action::ResultCode::ABORTED:
                          RCLCPP_ERROR(get_logger(),"Goal was aborted");
                          return;
                       case rclcpp_action::ResultCode::CANCELED:
                          RCLCPP_ERROR(get_logger(),"Goal was canceled");
                          return;
                       default:
                          RCLCPP_ERROR(get_logger(),"Unknown result code");
                          return;
                    }

                    RCLCPP_INFO(get_logger(),"Result received");
                    for(auto number : result.result->result)
                    {
                      RCLCPP_INFO(get_logger(), "%s", number.c_str());                     
                    }                 
                };
            
            auto goal_handle_future=action_client_->async_send_goal(goal,send_goal_options);
        }
    );
}


bool Switch::is_goal_done() const
{
    return goal_done_;
}


void Switch::message_info()
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");

  RCLCPP_INFO(this->get_logger(), "Requestor calls light!");
}

