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

#include <memory>

#include "mocap_control/ControllerNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#define WAIT_SECS 5.0

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto control_node = std::make_shared<mocap_control::ControllerNode>();

  auto start = control_node->now();
  while ((control_node->now() - start).seconds() < 1.0) {
    rclcpp::spin_some(control_node);
  }


  control_node->start_system();

  RCLCPP_INFO(control_node->get_logger(), "Wait %lf seconds or press Ctrl-C for exit");

  start = control_node->now();
  while ((control_node->now() - start).seconds() < WAIT_SECS) {
    rclcpp::spin_some(control_node);
  }

  rclcpp::shutdown();
}
