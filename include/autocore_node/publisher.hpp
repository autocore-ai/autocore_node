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
template <typename MessageT> class Publisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher)

  Publisher() : nodeType(NodeType::ZenohFlow) {}
  Publisher(const std::shared_ptr<rclcpp::Publisher<MessageT>> ros_pub)
  : nodeType(NodeType::ROS), p_ros_pub(ros_pub)
  {
  }

  void publish(const MessageT & msg)
  {
    message = msg;
    if (nodeType == NodeType::ROS) {
      p_ros_pub->publish(msg);
    } else if (nodeType == NodeType::ZenohFlow) {
    } else {
      throw "Unsupported autocore node type: " + nodeType;
    }
  }

  MessageT get() const { return message; }

  operator rclcpp::PublisherBase &() const { return *p_ros_pub; }

private:
  const NodeType nodeType = NodeType::Default;
  const std::shared_ptr<rclcpp::Publisher<MessageT>> p_ros_pub;
  MessageT message;
};
}  // namespace autocore
