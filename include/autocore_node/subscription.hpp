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
template <typename CallbackMessageT> class Subscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  Subscription()
  : nodeType(NodeType::ZenohFlow),
    callback_ptr(
      std::make_shared<rclcpp::AnySubscriptionCallback<CallbackMessageT, std::allocator<void>>>(
        std::make_shared<std::allocator<void>>()))
  {
  }
  Subscription(const std::shared_ptr<rclcpp::Subscription<CallbackMessageT>> ros_sub)
  : nodeType(NodeType::ROS), p_ros_sub(ros_sub)
  {
  }

  void set(const CallbackMessageT & msg, const rclcpp::MessageInfo msg_info = rclcpp::MessageInfo())
  {
    message = msg;
    if (nodeType == NodeType::ZenohFlow) {
      callback_ptr->dispatch(std::make_shared<CallbackMessageT>(message), msg_info);
    }
  }

  template <typename CallbackT> void setCallback(CallbackT && callback)
  {
    callback_ptr->set(std::forward<CallbackT>(callback));
  }

private:
  const NodeType nodeType;
  const std::shared_ptr<rclcpp::AnySubscriptionCallback<CallbackMessageT, std::allocator<void>>>
    callback_ptr;
  const std::shared_ptr<rclcpp::Subscription<CallbackMessageT>> p_ros_sub;
  CallbackMessageT message;
};
};  // namespace autocore
