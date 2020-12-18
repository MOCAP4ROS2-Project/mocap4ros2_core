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

#include "mocap_control/ControlledNode.hpp"

namespace mocap_control
{

using std::placeholders::_1;

ControlledNode::ControlledNode()
: node_(nullptr)
{}

void
ControlledNode::init(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  control_sub_ = node_->create_subscription<mocap_control_msgs::msg::Control>(
    "/mocap_control", rclcpp::QoS(100).reliable(),
    std::bind(&ControlledNode::control_callback, this, _1));

  control_pub_ = node_->create_publisher<mocap_control_msgs::msg::Control>(
    "/mocap_control", rclcpp::QoS(100).reliable());
}

void
ControlledNode::control_callback(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
  if (msg->mocap_component == node_->get_name()) {
    return;
  }

  switch (msg->control_type) {
    case mocap_control_msgs::msg::Control::START:
      {
        mocap_control_msgs::msg::Control msg_reply;
        msg_reply.control_type = mocap_control_msgs::msg::Control::ACK_START;
        msg_reply.mocap_component = node_->get_name();
        msg_reply.header.stamp = node_->now();

        control_pub_->publish(msg_reply);

        control_start();
      }
      break;

    case mocap_control_msgs::msg::Control::STOP:
      {
        mocap_control_msgs::msg::Control msg_reply;
        msg_reply.control_type = mocap_control_msgs::msg::Control::ACK_STOP;
        msg_reply.mocap_component = node_->get_name();
        msg_reply.header.stamp = node_->now();

        control_pub_->publish(msg_reply);

        control_stop();
      }
      break;

    default:
      break;
  }
}

void
ControlledNode::control_start()
{
}

void
ControlledNode::control_stop()
{
}

}  // namespace mocap_control
