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

#include "subscriber/observer.hpp"

using namespace robotis;

// https://en.cppreference.com/w/cpp/symbol_index/chrono_literals
using namespace std::chrono_literals;

Observer::Observer(const int32_t & qos_profile)
: Node("observer")
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");

  init_parameters();
  parameter_event_callback();

  // Set QoS profile
  auto qos = get_qos(qos_profile);

  // Rambda function (https://en.cppreference.com/w/cpp/language/lambda)
  // 'typename' specify that this is a type
  sub_ = this->create_subscription<examples_msgs::msg::Count>(
    "count",
    qos,
    [this](const typename examples_msgs::msg::Count::SharedPtr msg) -> void
      {
        static uint8_t blind_count = 0;

        if (blind_)
        {
          if (blind_count >= 10)
          {
            this->set_parameters({rclcpp::Parameter("blind", false)});
            this->get_parameter("blind", blind_);

            blind_count = 0;
          }

          RCLCPP_INFO(this->get_logger(), "Blind mode");

          blind_count++;
        }
        else
        {
          RCLCPP_INFO(
            this->get_logger(),
            "[%s] Observed: '%d'",
            comment_.c_str(), msg->count + offset_);
        }
      }
    );
}

void Observer::init_parameters()
{
  // Declare parameters that may be set on this node
  this->declare_parameter(
    "comment",
    rclcpp::ParameterValue("No comments"),
    rcl_interfaces::msg::ParameterDescriptor());
  this->declare_parameter(
    "blind",
    rclcpp::ParameterValue(false),
    rcl_interfaces::msg::ParameterDescriptor());
  this->declare_parameter(
    "offset",
    rclcpp::ParameterValue(0),
    rcl_interfaces::msg::ParameterDescriptor());
  this->declare_parameter(
    "favorite.numbers_integers",
    rclcpp::ParameterValue(std::vector<int64_t>({1, 2})),
    rcl_interfaces::msg::ParameterDescriptor());
  this->declare_parameter(
    "favorite.numbers_doubles",
    rclcpp::ParameterValue(std::vector<double>({1.0, 2.0})),
    rcl_interfaces::msg::ParameterDescriptor());

  // Get parameter from yaml
  this->get_parameter("comment", comment_);
  this->get_parameter("blind", blind_);
  this->get_parameter("offset", offset_);

  std::vector<int64_t> int_nums =
    this->get_parameter("favorite.numbers_integers").as_integer_array();
  std::vector<double> double_nums =
    this->get_parameter("favorite.numbers_doubles").as_double_array();
}

void Observer::parameter_event_callback()
{
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!parameters_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Setup callback for changes to parameters from parameter server.
  // (ref: http://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Parameter.html)
  auto param_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
      {
        for (auto & new_parameter : event->new_parameters)
        {
          RCLCPP_INFO(this->get_logger(), "new parameter name : %s", new_parameter.name.c_str());
        }

        for (auto & changed_parameter : event->changed_parameters)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "changed parameter name : %s",
            changed_parameter.name.c_str());

          if (changed_parameter.name == "blind")
          {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_bool();
            blind_ = value;
          }
          else if (changed_parameter.name == "offset")
          {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_int();
            offset_ = value;
          }
          else if (changed_parameter.name == "comment")
          {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_string();
            comment_ = value;
          }
        }

        for (auto & deleted_parameter : event->deleted_parameters)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "deleted parameter name : %s",
            deleted_parameter.name.c_str());
        }
      };

  parameter_event_sub_ = parameters_client_->on_parameter_event(param_event_callback);
}

rclcpp::QoS Observer::get_qos(const int32_t & qos_profile)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  if (qos_profile == 0)
  {
    qos = qos;
  }
  else if (qos_profile == 1)
  {
    qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  }
  else if (qos_profile == 2)
  {
    qos = rclcpp::QoS(rclcpp::ParametersQoS());
  }
  else if (qos_profile == 3)
  {
    qos = rclcpp::QoS(rclcpp::ServicesQoS());
  }
  else if (qos_profile == 4)
  {
    qos = rclcpp::QoS(rclcpp::ParameterEventsQoS());
  }
  else if (qos_profile == 5)
  {
    qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  }
  else
  {
    qos = qos;
  }

  return qos;
}
