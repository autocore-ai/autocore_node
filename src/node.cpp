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

#include <autocore_node/node.hpp>

namespace autocore
{
Node::Node(
  const std::string & node_name, const rclcpp::NodeOptions & options, const NodeType node_type)
: rclcpp::Node(node_name, options), nodeType(node_type)
{
}

Node::Node(const std::string & node_name, const NodeType node_type)
: rclcpp::Node(node_name), nodeType(node_type)
{
}

NodeType Node::GetNodeType(){ return nodeType; }

bool Node::IsROS() { return NodeType::ROS == nodeType; }

bool Node::IsZenohFlow() { return NodeType::ZenohFlow == nodeType; }

}  // namespace autocore
