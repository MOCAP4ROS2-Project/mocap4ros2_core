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

#include "mocap_camera_composer/mocap_camera_composer.hpp"
#include <string>
#include <vector>
#include <memory>

using std::min;
using std::max;
using std::string;
using std::map;
using std::stringstream;
using std::placeholders::_1;

void MocapCameraComposer::start_composer()
{
  marker_sub_ = camera_composer_node->create_subscription<mocap_msgs::msg::Markers>(
    tracked_frame_suffix_ + "/markers",
    100,
    std::bind(
      &MocapCameraComposer::marker_to_tf,
      this,
      _1));
}

/* void MocapCameraComposer::marker_to_tf(
    mocap_msgs::msg::Marker marker,
    int marker_num,
    const rclcpp::Time& frame_time) */
void MocapCameraComposer::marker_to_tf(const mocap_msgs::msg::Markers::SharedPtr markers_msg)
{
  int marker_id = 0;
  string tracked_frame;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  for (mocap_msgs::msg::Marker marker : markers_msg->markers) {
    // RCLCPP_INFO(camera_composer_node->get_logger(), "marker name %s", marker.marker_name);
    marker_id++;
    tf2::Transform transform;
    transform.setOrigin(
      tf2::Vector3(
        marker.translation.x / 1000,
        marker.translation.y / 1000,
        marker.translation.z / 1000)
    );
    transform.setRotation(tf2::Quaternion(0, 0, 0, 1));
    tracked_frame = tracked_frame_suffix_ + "/tf_marker_" + std::to_string(marker_id);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = markers_msg->header.stamp;
    tf_msg.header.frame_id = tf_ref_frame_id_;
    tf_msg.child_frame_id = tracked_frame;
    tf_msg.transform.translation.x = transform.getOrigin().x();
    tf_msg.transform.translation.y = transform.getOrigin().y();
    tf_msg.transform.translation.z = transform.getOrigin().z();
    tf_msg.transform.rotation.x = transform.getRotation().x();
    tf_msg.transform.rotation.y = transform.getRotation().y();
    tf_msg.transform.rotation.z = transform.getRotation().z();
    tf_msg.transform.rotation.w = transform.getRotation().w();
    transforms.push_back(tf_msg);
  }
  tf_broadcaster_->sendTransform(transforms);
}

MocapCameraComposer::MocapCameraComposer()
{
  camera_composer_node = rclcpp::Node::make_shared("camera_composer_node");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(camera_composer_node);
  tracked_frame_suffix_ = "vicon";
  tf_ref_frame_id_ = "vicon_world";
}
