#pragma once

#include <behaviortree_cpp/action_node.h>

namespace r2_bt
{

class SetDebugStatus : public BT::SyncActionNode
{
public:
  SetDebugStatus(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace r2_bt
