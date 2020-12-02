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
// Author: Lorena Bajo Rebollo <lorena.bajo@urjc.es>

#include "mocap_imu_composer/mocap_imu_composer.hpp"
#include <string>
#include <vector>
#include <memory>

using std::min;
using std::max;
using std::string;
using std::map;
using std::stringstream;
using std::placeholders::_1;

void MocapImuComposer::start_composer()
{
  imu_composer_node->get_parameter<int>("imu_num", total_imus);
  //imu_composer_node->get_parameter<std::vector<imuCamera>>("imu_array", imuVectorInit);

  imu_sub_ = imu_composer_node->create_subscription<sensor_msgs::msg::Imu>("/imu",
    100, std::bind(&MocapImuComposer::process_imu_msgs, this, _1));
}

void MocapImuComposer::process_imu_msgs(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{    
    //RCLCPP_INFO(imu_composer_node->get_logger(), "imu name %s", imu_msg->header.frame_id);
  
    imuCamera imuCameraAux;
    for (imuCamera imu : imuVector) {
      if (imu.name == imu_msg->header.frame_id){
        imuCameraAux = imu;
      }
    }

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = imu_msg->header.stamp;
    transform.header.frame_id = "imu_world";
    transform.child_frame_id = imu_msg->header.frame_id;
    transform.transform.translation.x = imuCameraAux.x;
    transform.transform.translation.y = imuCameraAux.y;
    transform.transform.translation.z = imuCameraAux.z;
    transform.transform.rotation.x = imu_msg->orientation.x;
    transform.transform.rotation.y = imu_msg->orientation.y;
    transform.transform.rotation.z = imu_msg->orientation.z;
    transform.transform.rotation.w = imu_msg->orientation.w;
    
    //tf_broadcaster_.sendTransform(transform);

}

MocapImuComposer::MocapImuComposer(const rclcpp::NodeOptions node_options)
{
  std::vector<imuCamera> imuVectorInit;
  imuVectorInit[0].name = "imu0000";
  imuVectorInit[0].x = 0;
  imuVectorInit[0].y = 0;
  imuVectorInit[0].z = 0;
  imu_composer_node = rclcpp::Node::make_shared("mocap_imu_composer");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(imu_composer_node);
  imu_composer_node->declare_parameter<int>("imu_num", 0);
  //imu_composer_node->declare_parameter<std::vector<imuCamera>>("imu_array", imuVectorInit);
}
