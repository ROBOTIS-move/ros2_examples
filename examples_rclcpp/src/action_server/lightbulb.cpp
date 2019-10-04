#include "action_server/lightbulb.hpp"

using namespace robotis;

LightBulb::LightBulb()
: Node("light")
{
  using namespace std::placeholders;

  print_message_info();

  this->action_server_ = rclcpp_action::create_server<examples_msgs::action::Led>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "Led_on",
    std::bind(&LightBulb::handle_goal, this, _1, _2),
    std::bind(&LightBulb::handle_cancel, this, _1),
    std::bind(&LightBulb::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse LightBulb::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const examples_msgs::action::Led::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request %d", goal->numbers);
  (void)uuid;

  if (goal->numbers > 6)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LightBulb::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LightBulb::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&LightBulb::execute, this, _1), goal_handle}.detach();
}

void LightBulb::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<examples_msgs::action::Led>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<examples_msgs::action::Led::Feedback>();
  auto & process = feedback->process;

  auto result = std::make_shared<examples_msgs::action::Led::Result>();

  std::string lantern = "[ ][ ][ ][ ][ ]";

  for (int i = 0; (i < goal->numbers) && rclcpp::ok(); ++i)
  {
    if (goal_handle->is_canceling())
    {
      result->result = process;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal Canceled");
      return;
    }

    lantern[3 * i + 1] = 'O';
    process.push_back(lantern);

    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Publish Feedback");

    loop_rate.sleep();
  }

  if (rclcpp::ok())
  {
    result->result = process;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Achieve the goal");
  }
}

void LightBulb::print_message_info()
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");
  RCLCPP_INFO(
  this->get_logger(),
  "ros2 action_server call /Led_on examples_msgs/action/Led \"{numbers : 5}\"");
}
