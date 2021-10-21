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

#include <autocore_node/autocore_node.hpp>

AutoCoreNode::AutoCoreNode(const std::string & node_name, const rclcpp::NodeOptions & options, const NodeType type)
: Node(node_name, options), node_type(type)
{
}

AutoCoreNode::AutoCoreNode(const std::string & node_name, const NodeType type)
: Node(node_name), node_type(type)
{
}

bool AutoCoreNode::IsROS() { return NodeType::ROS == node_type; }

bool AutoCoreNode::IsZenohFlow() { return NodeType::ZenohFlow == node_type; }
