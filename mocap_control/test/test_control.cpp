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
#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "mocap_control_msgs/msg/control.hpp"
#include "mocap_control/ControlledLifecycleNode.hpp"
#include "mocap_control/ControllerNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

class MocapSystem1 : public mocap_control::ControlledLifecycleNode
{
public:
  explicit MocapSystem1(const std::string & id)
  : ControlledLifecycleNode(id) {}

  void control_start(const mocap_control_msgs::msg::Control::SharedPtr msg) override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Starting capture");
    capturing_ = true;
    session_id_ = msg->capture_session_id;
    last_msg_ = *msg;
  }

  void control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg) override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Starting capture");
    capturing_ = false;
    session_id_ = "";
    last_msg_ = *msg;
  }

  bool capturing_ {false};
  std::string session_id_;
  mocap_control_msgs::msg::Control last_msg_;
};

TEST(Controltests, test_sync)
{
  auto node_1 = std::make_shared<MocapSystem1>("mocap_system_1");
  auto node_2 = std::make_shared<MocapSystem1>("mocap_system_2");
  auto control_node = std::make_shared<mocap_control::ControllerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_1->get_node_base_interface());
  executor.add_node(node_2->get_node_base_interface());
  executor.add_node(control_node);

  node_1->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_2->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  auto start = control_node->now();
  while ((control_node->now() - start).seconds() < 0.1) {
    executor.spin_some();
  }

  // Capturing session 1: all systems
  ASSERT_EQ(node_1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_2->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_FALSE(node_1->capturing_);
  ASSERT_FALSE(node_2->capturing_);

  control_node->start_system("session_1", {});

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_EQ(node_1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_2->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(node_1->capturing_);
  ASSERT_TRUE(node_2->capturing_);
  ASSERT_EQ(node_1->session_id_, "session_1");
  ASSERT_EQ(node_2->session_id_, "session_1");

  control_node->stop_system();

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_EQ(node_1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_2->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_FALSE(node_1->capturing_);
  ASSERT_FALSE(node_2->capturing_);

  // Capturing session 2: system 1 only
  control_node->start_system("session_2", {"mocap_system_1"});

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_EQ(node_1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_2->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(node_1->capturing_);
  ASSERT_FALSE(node_2->capturing_);
  ASSERT_EQ(node_1->session_id_, "session_2");

  control_node->stop_system();

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_EQ(node_1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_2->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_FALSE(node_1->capturing_);
  ASSERT_FALSE(node_2->capturing_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  rclcpp::Rate(1).sleep();
  return result;
}
