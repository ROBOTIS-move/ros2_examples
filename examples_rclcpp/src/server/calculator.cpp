/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Pyo */

#include "server/calculator.hpp"

using namespace robotis;

Calculator::Calculator()
: Node("calculator")
{
  message_info();

  // Create a callback function for when service requests are received.
  auto calculator_callback =
    [this](
      const std::shared_ptr<examples_msgs::srv::Calculation::Request> request,
      std::shared_ptr<examples_msgs::srv::Calculation::Response> response) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Incoming request");
        RCLCPP_INFO(this->get_logger(),
          "a = %d, b = %d, calculator(a %s b)",
          request->a, request->b,
          request->arithmetic_operator.c_str());

        response->result = calculation(request->a, request->b, request->arithmetic_operator);
        
      };
  
  // Create a service that will use the callback function to handle requests.
  srv_ = create_service<examples_msgs::srv::Calculation>("calculate", calculator_callback);
}

void Calculator::message_info()
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");

  RCLCPP_INFO(this->get_logger(), "Calculator is ready to get calculation!");
  RCLCPP_INFO(this->get_logger(), "Please use CLI or Service Caller in rqt to call service server");
  RCLCPP_INFO(this->get_logger(), "Examples of CLI");
  RCLCPP_INFO(
    this->get_logger(),
    "ros2 service call /calculate examples_msgs/srv/Calculation \"{a: 1, b: 2, arithmetic_operator: plus}\"");
}

int32_t Calculator::calculation(
  const int32_t & a,
  const int32_t & b,
  const std::string & arithmetic_operator)
{
  if (arithmetic_operator == "plus")
  {
    return a + b;
  }
  else if (arithmetic_operator == "minus")
  {
    return a - b;
  }
  else if (arithmetic_operator == "multiply")
  {
    return a * b;
  }
  else if (arithmetic_operator == "division")
  {
    try
    {
      if (b == 0) throw b;

      return a / b;
    }
    catch(int32_t denominator)
    {
      RCLCPP_ERROR(this->get_logger(), "Can't divide by zero");
      return 0;
    }
  }
  else
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Please make sure arithmetic_operator(plus, minus, multiply, division)");
    return 0;
  }

  return 0;
}
