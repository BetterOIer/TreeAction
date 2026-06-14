#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace r2_bt
{

class IsStringNonEmpty : public BT::ConditionNode
{
public:
  IsStringNonEmpty(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace r2_bt
