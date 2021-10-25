// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <autocore_node/type.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autocore
{
template <typename MessageT, typename AllocatorT = std::allocator<void>> class Publisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher)

  Publisher() : nodeType(NodeType::ZenohFlow) {}
  Publisher(const std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> ros_pub)
  : nodeType(NodeType::ROS), p_ros_pub(ros_pub)
  {
  }

  void publish(const MessageT & msg)
  {
    if (nodeType == NodeType::ROS) {
      p_ros_pub->publish(msg);
    } else if (nodeType == NodeType::ZenohFlow) {
      // TODO: pub in Zenoh Flow
    } else {
      throw "Unsupported autocore node type: " + nodeType;
    }
  }

private:
  const std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT>> p_ros_pub;
  const NodeType nodeType;
};
};  // namespace autocore
