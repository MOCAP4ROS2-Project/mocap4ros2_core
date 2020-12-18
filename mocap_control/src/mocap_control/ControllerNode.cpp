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

#include "mocap_control_msgs/msg/control.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mocap_control/ControllerNode.hpp"

namespace mocap_control
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ControllerNode::ControllerNode()
: Node("mocap_controller")
{
  control_sub_ = create_subscription<mocap_control_msgs::msg::Control>(
    "/mocap_control", rclcpp::QoS(100).reliable(),
    std::bind(&ControllerNode::control_callback, this, _1));

  control_pub_ = create_publisher<mocap_control_msgs::msg::Control>(
    "/mocap_control", rclcpp::QoS(100).reliable());
}

void
ControllerNode::control_callback(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
  if (msg->mocap_component == get_name()) {
    return;
  }

  switch (msg->control_type) {
    case mocap_control_msgs::msg::Control::ACK_START:
      {
        auto elapsed = now() - msg->header.stamp;

        if (elapsed < 1ms) {
          RCLCPP_INFO(
            get_logger(),
            "[%s] start elapsed = %lf secs", msg->mocap_component.c_str(), elapsed.seconds());
        } else if (elapsed < 200ms) {
          RCLCPP_WARN(
            get_logger(),
            "[%s] start elapsed = %lf secs", msg->mocap_component.c_str(), elapsed.seconds());
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "[%s] start elapsed = %lf secs", msg->mocap_component.c_str(), elapsed.seconds());
        }
      }
      break;

    case mocap_control_msgs::msg::Control::ACK_STOP:
      {
        auto elapsed = now() - msg->header.stamp;

        if (elapsed < 1ms) {
          RCLCPP_INFO(
            get_logger(),
            "[%s] stop elapsed = %lf secs", msg->mocap_component.c_str(), elapsed.seconds());
        } else if (elapsed < 200ms) {
          RCLCPP_WARN(
            get_logger(),
            "[%s] stop = %lf secs", msg->mocap_component.c_str(), elapsed.seconds());
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "[%s] stop = %lf secs", msg->mocap_component.c_str(), elapsed.seconds());
        }
      }
      break;

    default:
      break;
  }
}

void
ControllerNode::start_system()
{
  mocap_control_msgs::msg::Control msg;
  msg.control_type = mocap_control_msgs::msg::Control::START;
  msg.mocap_component = get_name();
  msg.header.stamp = now();

  control_pub_->publish(msg);
}

void
ControllerNode::stop_system()
{
  mocap_control_msgs::msg::Control msg;
  msg.control_type = mocap_control_msgs::msg::Control::STOP;
  msg.mocap_component = get_name();
  msg.header.stamp = now();

  control_pub_->publish(msg);
}

}  // namespace mocap_control
