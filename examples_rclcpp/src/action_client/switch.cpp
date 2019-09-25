#include "action_client/switch.hpp"
#include <inttypes.h>

using namespace robotis;

Switch::Switch()
: Node("action_client"), goal_done_(false)
{
    this->action_client_=rclcpp_action::create_client<examples_msgs::action::Led>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "Led_on");

    this->timer_=this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Switch::send_goal,this)
    );
}


bool Switch::is_goal_done() const
{
    return goal_done_;
}

void Switch::send_goal()
{
    using namespace std::placeholders;

    timer_->cancel();

    goal_done_=false;

    if(!Switch::action_client_){
        RCLCPP_INFO(get_logger(), "Action client not initialized");
    }

    if(!Switch::action_client_->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
        goal_done_=true;
        return;
    }

    auto goal=examples_msgs::action::Led::Goal();
    goal.numbers=5;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options=rclcpp_action::Client<examples_msgs::action::Led>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Switch::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&Switch::feedback_callback, this, _1, _2);
    send_goal_options.result_callback=std::bind(&Switch::result_callback,this,_1);
    auto goal_handle_future=action_client_->async_send_goal(goal,send_goal_options);
}

void Switch::goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr>future)
{
    auto goal_handle=future.get();
    if(!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");    
    }
}

void Switch::feedback_callback(
    rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::SharedPtr,
    const std::shared_ptr<const examples_msgs::action::Led::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(),"Next number in sequence received: %" PRId64,feedback->process.back());
    }

void Switch::result_callback(const rclcpp_action::ClientGoalHandle<examples_msgs::action::Led>::WrappedResult & result)
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

    //RCLCPP_INFO(get_logger(),"Result received");
    for(auto number : result.result->result)
    {
        RCLCPP_INFO(this->get_logger(), "%" PRId64, number);
    }
}