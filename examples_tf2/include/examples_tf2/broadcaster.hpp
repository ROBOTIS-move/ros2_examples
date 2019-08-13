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

#ifndef EXAMPLES_TF2_BROADCASTER_HPP_
#define EXAMPLES_TF2_BROADCASTER_HPP_

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace robotis
{
class Arm : public rclcpp::Node
{
 public:
  explicit Arm();

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> tf_stamped_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;

  bool move_flag_;
};
} // robotis
#endif // EXAMPLES_TF2_BROADCASTER_HPP_
