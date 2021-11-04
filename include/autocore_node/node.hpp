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

#include <autocore_node/publisher.hpp>
#include <autocore_node/subscription.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autocore
{
class Node : public rclcpp::Node
{
public:
  explicit Node(const std::string & node_name, const NodeType node_type = NodeType::ZenohFlow);
  explicit Node(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    const NodeType node_type = NodeType::ROS);

  NodeType GetNodeType();
  bool IsROS();
  bool IsZenohFlow();

  template <typename MessageT>
  std::shared_ptr<autocore::Publisher<MessageT>> create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos, const bool force_ros_type = false)
  {
    if (force_ros_type || IsROS()) {
      return std::make_shared<autocore::Publisher<MessageT>>(
        rclcpp::Node::create_publisher<MessageT>(topic_name, qos));
    } else if (IsZenohFlow()) {
      return std::make_shared<autocore::Publisher<MessageT>>();
    } else {
      throw "Unsupported autocore node type: " + GetNodeType();
    }
  }

  template <typename MessageT, typename CallbackT>
  std::shared_ptr<
    autocore::Subscription<typename rclcpp::subscription_traits::has_message_type<CallbackT>::type>>
  create_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const bool force_ros_type = false)
  {
    if (force_ros_type || IsROS()) {
      return std::make_shared<autocore::Subscription<
        typename rclcpp::subscription_traits::has_message_type<CallbackT>::type>>(
        rclcpp::Node::create_subscription<MessageT, CallbackT>(
          topic_name, qos, std::forward<CallbackT>(callback)));
    } else if (IsZenohFlow()) {
      auto p_sub = std::make_shared<autocore::Subscription<
        typename rclcpp::subscription_traits::has_message_type<CallbackT>::type>>();
      p_sub->setCallback(callback);
      return p_sub;
    } else {
      throw "Unsupported autocore node type: " + GetNodeType();
    }
  }

protected:
  const NodeType nodeType;
};
}  // namespace autocore
