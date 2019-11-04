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

#ifndef EXAMPLES_MESSAGE_FILTER_MESSAGE_FILTER_HPP_
#define EXAMPLES_MESSAGE_FILTER_MESSAGE_FILTER_HPP_

// Load chrono
#include <chrono>
// Load smart pointer
#include <memory>
// Load std::string
#include <string>
// Load move, forward, tuple and etc (https://en.cppreference.com/w/cpp/header/utility)
#include <utility>

// Load user-defined msgs
#include <examples_msgs/msg/count.hpp>
// Load message_filters
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
// Load C++ ROS2 Client Library (http://docs.ros2.org/dashing/api/rclcpp/index.html)
#include <rclcpp/rclcpp.hpp>


namespace robotis
{
class MsgFilters : public rclcpp::Node
{
  typedef message_filters::sync_policies::ApproximateTime<
    examples_msgs::msg::Count,
    examples_msgs::msg::Count> SyncPolicySensorData;

  typedef message_filters::Synchronizer<SyncPolicySensorData> SynchronizerSensorData;

 public:
  // 'explicit' makes block data conversion
  explicit MsgFilters(
    const uint32_t & queue_size,
    const std::string & topic_sub_1,
    const std::string & topic_sub_2)
  : Node("msg_filters")
  {
    RCLCPP_INFO(this->get_logger(), "msg_filters node init");

    synced_data_ = std::make_shared<SynchronizerSensorData>(queue_size);

    msg_ftr_sub_1_ =
      std::make_shared<
        message_filters::Subscriber<examples_msgs::msg::Count>>(
          this,
          topic_sub_1);

    msg_ftr_sub_2_ =
      std::make_shared<
        message_filters::Subscriber<examples_msgs::msg::Count>>(
          this,
          topic_sub_2);

    // connect message filters to synchronizer
    synced_data_->connectInput(
      *msg_ftr_sub_1_,
      *msg_ftr_sub_2_);

    using namespace std::chrono_literals;
    // synced_data_->setMaxIntervalDuration(rclcpp::Duration(18ms));
    // synced_data_->setInterMessageLowerBound(0, rclcpp::Duration(36ms));
    // synced_data_->setInterMessageLowerBound(1, rclcpp::Duration(18ms));
    // synced_data_->setInterMessageLowerBound(2, rclcpp::Duration(25ms));

    synced_data_->registerCallback(
      std::bind(
        &MsgFilters::synced_data_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

 private:
  void synced_data_callback(
    const std::shared_ptr<examples_msgs::msg::Count const> & msg_1,
    const std::shared_ptr<examples_msgs::msg::Count const> & msg_2)
  {
    static rclcpp::Time last_t;
    rclcpp::Duration duration(this->now().nanoseconds() - last_t.nanoseconds());

    RCLCPP_INFO(
      this->get_logger(),
      "%.2f [millis] Get Msg : %d, %d",
      duration.nanoseconds() / 1.0E+6,
      msg_1->count,
      msg_2->count);

    last_t = this->now();
  }

  std::shared_ptr<
    message_filters::Subscriber<examples_msgs::msg::Count>> msg_ftr_sub_1_;
  std::shared_ptr<
    message_filters::Subscriber<examples_msgs::msg::Count>> msg_ftr_sub_2_;

  std::shared_ptr<SynchronizerSensorData> synced_data_;
};
} // robotis
#endif // EXAMPLES_MESSAGE_FILTER_MESSAGE_FILTER_HPP_
