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
// Author: Lorena Bajo <lorena.bajo@urjc.es>

#ifndef MOCAP_IMU_COMPOSER__MOCAP_IMU_COMPOSER_HPP_
#define MOCAP_IMU_COMPOSER__MOCAP_IMU_COMPOSER_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include "rclcpp/time.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/node_interfaces/node_logging.hpp"

struct imuCamera {
  std::string name;
  float x;
  float y;
  float z;
} ;

class MocapImuComposer
{
public:
  MocapImuComposer(
    const rclcpp::NodeOptions options =
    rclcpp::NodeOptions().parameter_overrides(
      std::vector<rclcpp::Parameter> {
    rclcpp::Parameter("use_sim_time", true)
  }));
  void start_composer();

private:
  rclcpp::Node::SharedPtr imu_composer_node;
  int total_imus;
  std::vector<imuCamera> imuVector;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  void process_imu_msgs(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
};
#endif  // MOCAP_IMU_COMPOSER__MOCAP_IMU_COMPOSER_HPP_
