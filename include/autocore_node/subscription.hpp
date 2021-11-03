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
template <
  typename CallbackMessageT,
  typename AllocatorT = std::allocator<void>,
  typename MessageMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>>
class Subscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  Subscription() : nodeType(NodeType::ZenohFlow) {}
  Subscription(const std::shared_ptr<
               rclcpp::Subscription<CallbackMessageT, AllocatorT, MessageMemoryStrategyT>> ros_sub)
  : nodeType(NodeType::ROS), p_ros_sub(ros_sub)
  {
  }

  void set(const CallbackMessageT & msg)
  {
    message = msg;
    if (nodeType == NodeType::ZenohFlow) {
      rclcpp::MessageInfo msg_info;
      any_subscription_callback.dispatch(std::make_shared<CallbackMessageT>(message), msg_info);
    }
  }

  template <typename CallbackT> void setCallback(CallbackT && callback)
  {
    any_subscription_callback.set(std::forward<CallbackT>(callback));
  }

private:
  const std::shared_ptr<rclcpp::Subscription<CallbackMessageT, AllocatorT, MessageMemoryStrategyT>>
    p_ros_sub;
  const NodeType nodeType;
  const rclcpp::AnySubscriptionCallback<CallbackMessageT, AllocatorT> any_subscription_callback;
  CallbackMessageT message;
};
};  // namespace autocore
