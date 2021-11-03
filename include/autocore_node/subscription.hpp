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
    rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>,
  typename SharedPtrCallback = std::function<void(const std::shared_ptr<CallbackMessageT>)>>
class Subscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  Subscription() : nodeType(NodeType::ZenohFlow), any_subscription_callback_(allocator_) {}
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
      // any_subscription_callback_.dispatch(std::make_shared<CallbackMessageT>(message), msg_info);
    }
  }

  template <
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_arguments<CallbackT, SharedPtrCallback>::value>::type * =
      nullptr>
  void setCallback(CallbackT && callback)
  {
    any_subscription_callback_.set(callback);
  }

private:
  const std::shared_ptr<rclcpp::Subscription<CallbackMessageT, AllocatorT, MessageMemoryStrategyT>>
    p_ros_sub;
  const NodeType nodeType;
  std::shared_ptr<std::allocator<void>> allocator_;
  const rclcpp::AnySubscriptionCallback<CallbackMessageT, std::allocator<void>>
    any_subscription_callback_;
  CallbackMessageT message;
};
};  // namespace autocore
