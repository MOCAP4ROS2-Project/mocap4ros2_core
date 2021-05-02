// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOCAP_CONTROL__CONTROLLEDLIFECYCLENODE_HPP_
#define MOCAP_CONTROL__CONTROLLEDLIFECYCLENODE_HPP_

#include <string>

#include "mocap_control_msgs/msg/control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mocap_control
{
class ControlledLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControlledLifecycleNode(const std::string & system_id);

protected:
  virtual void control_start(const mocap_control_msgs::msg::Control::SharedPtr msg) {(void)msg;}
  virtual void control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg) {(void)msg;}

private:
  void control_callback(const mocap_control_msgs::msg::Control::SharedPtr msg);

  rclcpp::Subscription<mocap_control_msgs::msg::Control>::SharedPtr control_sub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_control_msgs::msg::Control>::SharedPtr control_pub_;
};

}  // namespace mocap_control

#endif  // MOCAP_CONTROL__CONTROLLEDLIFECYCLENODE_HPP_
