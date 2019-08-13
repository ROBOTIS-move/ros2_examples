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

#include "client/requestor.hpp"

using namespace robotis;

// https://en.cppreference.com/w/cpp/symbol_index/chrono_literals
using namespace std::chrono_literals;

Requestor::Requestor(
  const int32_t a,
  const int32_t b,
  std::string & arithmetic_operator)
: Node("requestor")
{
  message_info();

  client_ = this->create_client<examples_msgs::srv::Calculation>("calculate");

  while (!client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  auto request = std::make_shared<examples_msgs::srv::Calculation::Request>();
  request->a = a;
  request->b = b;
  request->arithmetic_operator = arithmetic_operator;

  // We give the async_send_request() method a callback
  // that will get executed once the response is received.
  // This way we can return immediately from this method and allow other work to be done
  // by the executor in `spin` while waiting for the response.
  using ServiceResponseFuture =
    rclcpp::Client<examples_msgs::srv::Calculation>::SharedFuture;

  auto response_received_callback =
    [this](ServiceResponseFuture future)
      {
        RCLCPP_INFO(this->get_logger(), "Result of calculation: %d", future.get()->result);
        rclcpp::shutdown();
      };

  auto future_result = client_->async_send_request(request, response_received_callback);
}

void Requestor::message_info()
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");

  RCLCPP_INFO(this->get_logger(), "Requestor calls calculator!");
}
