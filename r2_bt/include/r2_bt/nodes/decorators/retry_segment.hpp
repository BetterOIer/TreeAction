#pragma once

#include <behaviortree_cpp/decorator_node.h>

namespace r2_bt
{

class RetrySegment : public BT::DecoratorNode
{
public:
  RetrySegment(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  void halt() override;

private:
  BT::NodeStatus tick() override;

  int try_count_ = 0;
};

}  // namespace r2_bt
