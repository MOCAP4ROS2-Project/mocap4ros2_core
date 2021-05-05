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

#include <string>

#include "mocap_control_msgs/Control.h"
#include "mocap_control_msgs/MocapInfo.h"

#include "ros/ros.h"

#include "mocap_control/ControlledLifecycleNode.hpp"

namespace mocap_control
{

ControlledLifecycleNode::ControlledLifecycleNode(const std::string & system_id)
: nh_(), system_id_(system_id), state_(UNCONFIGURED)
{
  mocap_control_sub_ = nh_.subscribe("mocap_control", 1, &ControlledLifecycleNode::control_callback, this);
  mocap_control_pub_ = nh_.advertise<mocap_control_msgs::Control>("mocap_control", 100);
  mocap_info_pub_ = nh_.advertise<mocap_control_msgs::MocapInfo>("mocap_environment", 100, true);
}

bool
ControlledLifecycleNode::trigger_transition(Transition transition)
{
  switch (transition) {
    case CONFIGURE:
      if (state_ == UNCONFIGURED) {
        if (on_configure()) {
          state_ = INACTIVE;
        }
      } else {
        ROS_ERROR("CONFIGURE transition from state %d", state_);
        return false;
      }
      break;
    case ACTIVATE:
      if (state_ == INACTIVE) {
        if (on_activate()) {
          state_ = ACTIVE;
        }
      } else {
        ROS_ERROR("ACTIVATE transition from state %d", state_);
        return false;
      }
      break;
    case DEACTIVATE:
      if (state_ == ACTIVE) {
        if (on_deactivate()) {
          state_ = INACTIVE;
        }
      } else {
        ROS_ERROR("DEACTIVATE transition from state %d", state_);
        return false;
      }
      break;
  }

  return true;
}

bool
ControlledLifecycleNode::on_configure()
{
  mocap_control_msgs::MocapInfo msg;
  msg.system_source = ros::this_node::getName();
  msg.topics.assign(topics_.begin(), topics_.end());

  mocap_info_pub_.publish(msg);

  return true;
}

bool
ControlledLifecycleNode::on_activate()
{
  return true;
}

bool
ControlledLifecycleNode::on_deactivate()
{
  return true;
}


void
ControlledLifecycleNode::control_callback(const mocap_control_msgs::Control::ConstPtr & msg)
{
  if (!msg->capture_systems.empty() &&
    std::find(msg->capture_systems.begin(), msg->capture_systems.end(), ros::this_node::getName()) ==
    msg->capture_systems.end())
  {
    return;
  }

  switch (msg->control_type) {
    case mocap_control_msgs::Control::START:
      if (state_ == INACTIVE) {
        trigger_transition(ACTIVATE);
        mocap_control_msgs::Control msg_reply;
        msg_reply.control_type = mocap_control_msgs::Control::ACK_START;
        msg_reply.stamp = ros::Time::now();
        msg_reply.system_source = ros::this_node::getName();
        mocap_control_pub_.publish(msg_reply);

        control_start(msg);
      } else {
        ROS_WARN_STREAM(
          "Activation requested in state " << state_);
      }
      break;

    case mocap_control_msgs::Control::STOP:
      if (state_ == ACTIVE) {
        trigger_transition(DEACTIVATE);
        mocap_control_msgs::Control msg_reply;
        msg_reply.control_type = mocap_control_msgs::Control::ACK_STOP;
        msg_reply.stamp = ros::Time::now();
        msg_reply.system_source =ros::this_node::getName();
        mocap_control_pub_.publish(msg_reply);

        control_stop(msg);
      } else {
        ROS_WARN_STREAM(
          "Deactivation requested in state " << state_);
      }
      break;

    default:
      break;
  }
}

}  // namespace mocap_control
