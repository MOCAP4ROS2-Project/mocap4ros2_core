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

#include "mocap_control/ControlledNode.hpp"
#include "mocap_control/ControlledLifecycleNode.hpp"
#include "mocap_control/ControllerNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

class MocapComponentTest : public rclcpp::Node, public mocap_control::ControlledNode
{
public:
  explicit MocapComponentTest(const std::string & name)
  : Node(name), started_(false)
  {
  }

  void start()
  {
    init(shared_from_this());
  }

  void control_start() override
  {
    started_ = true;
  }

  void control_stop() override
  {
    started_ = false;
  }

  bool started_;
};

class MocapLFComponentTest
  : public rclcpp_lifecycle::LifecycleNode,
  public mocap_control::ControlledLifecycleNode
{
public:
  explicit MocapLFComponentTest(const std::string & name)
  : LifecycleNode(name), started_(false)
  {
  }

  void start()
  {
    init(shared_from_this());
  }

  void control_start() override
  {
    started_ = true;
  }

  void control_stop() override
  {
    started_ = false;
  }

  bool started_;
};

TEST(Controltests, test_sync)
{
  auto node_1 = std::make_shared<MocapComponentTest>("system_1");
  auto node_2 = std::make_shared<MocapComponentTest>("system_2");
  auto control_node = std::make_shared<mocap_control::ControllerNode>();

  node_1->start();
  node_2->start();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_1);
  executor.add_node(node_2);
  executor.add_node(control_node);

  auto start = control_node->now();
  while ((control_node->now() - start).seconds() < 0.1) {
    executor.spin_some();
  }

  ASSERT_FALSE(node_1->started_);
  ASSERT_FALSE(node_2->started_);

  control_node->start_system();

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_TRUE(node_1->started_);
  ASSERT_TRUE(node_2->started_);

  control_node->stop_system();

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_FALSE(node_1->started_);
  ASSERT_FALSE(node_2->started_);
}

TEST(Controltests, test_sync_lifecycle)
{
  auto node_1 = std::make_shared<MocapLFComponentTest>("system_1");
  auto node_2 = std::make_shared<MocapLFComponentTest>("system_2");
  auto control_node = std::make_shared<mocap_control::ControllerNode>();

  node_1->start();
  node_2->start();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_1->get_node_base_interface());
  executor.add_node(node_2->get_node_base_interface());
  executor.add_node(control_node);

  auto start = control_node->now();
  while ((control_node->now() - start).seconds() < 0.1) {
    executor.spin_some();
  }

  ASSERT_FALSE(node_1->started_);
  ASSERT_FALSE(node_2->started_);

  control_node->start_system();

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_TRUE(node_1->started_);
  ASSERT_TRUE(node_2->started_);

  control_node->stop_system();

  start = control_node->now();
  while ((control_node->now() - start).seconds() < 1) {
    executor.spin_some();
  }

  ASSERT_FALSE(node_1->started_);
  ASSERT_FALSE(node_2->started_);
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
