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

  template <
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = autocore::Publisher<MessageT>,
    typename PubRclcppT = rclcpp::Publisher<MessageT, AllocatorT>>
  std::shared_ptr<PublisherT> create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
  {
    if (IsROS()) {
      return std::make_shared<PublisherT>(
        rclcpp::Node::create_publisher<MessageT, AllocatorT, PubRclcppT>(topic_name, qos, options));
    } else if (IsZenohFlow()) {
      return std::make_shared<PublisherT>();
    } else {
      throw "Unsupported autocore node type: " + GetNodeType();
    }
  }

  template <
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename CallbackMessageT =
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename SubscriptionT = autocore::Subscription<CallbackMessageT, AllocatorT>,
    typename MessageMemoryStrategyT =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>,
    typename SubRclcppT =
      rclcpp::Subscription<CallbackMessageT, AllocatorT, MessageMemoryStrategyT>>
  std::shared_ptr<SubscriptionT> create_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
      (MessageMemoryStrategyT::create_default()))
  {
    if (IsROS()) {
      return std::make_shared<SubscriptionT>(rclcpp::Node::create_subscription<
                                             MessageT,
                                             CallbackT,
                                             AllocatorT,
                                             CallbackMessageT,
                                             SubRclcppT,
                                             MessageMemoryStrategyT>(
        topic_name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat));
    } else if (IsZenohFlow()) {
      //TODO: Implete callback in Zenoh Flow.
      return std::make_shared<SubscriptionT>();
    } else {
      throw "Unsupported autocore node type: " + GetNodeType();
    }
  }

protected:
  const NodeType nodeType;
};
};  // namespace autocore
