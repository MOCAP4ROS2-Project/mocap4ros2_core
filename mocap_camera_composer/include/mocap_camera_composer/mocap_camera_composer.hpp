// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: David Vargas Frutos <david.vargas@urjc.es>

#ifndef MOCAP_CAMERA_COMPOSER__MOCAP_CAMERA_COMPOSER_HPP_
#define MOCAP_CAMERA_COMPOSER__MOCAP_CAMERA_COMPOSER_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include "rclcpp/time.hpp"

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/node_interfaces/node_logging.hpp"

class MocapCameraComposer
{
public:
  std::string tracked_frame_suffix_;
  std::string tf_ref_frame_id_;
  MocapCameraComposer();
  void start_composer();

private:
  rclcpp::Node::SharedPtr camera_composer_node;
  rclcpp::Subscription<mocap_msgs::msg::Markers>::SharedPtr marker_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // void marker_to_tf(mocap_msgs::msg::Marker marker, int marker_num,
  // const rclcpp::Time& frame_time);
  void marker_to_tf(const mocap_msgs::msg::Markers::SharedPtr markers_msg);
};
#endif  // MOCAP_CAMERA_COMPOSER__MOCAP_CAMERA_COMPOSER_HPP_
