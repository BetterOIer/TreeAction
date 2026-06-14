#include "r2_bt/nodes/conditions/is_string_non_empty.hpp"

namespace r2_bt
{

IsStringNonEmpty::IsStringNonEmpty(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList IsStringNonEmpty::providedPorts()
{
  return {
    BT::InputPort<std::string>("value", "String to test — SUCCESS if non-empty and present"),
  };
}

BT::NodeStatus IsStringNonEmpty::tick()
{
  auto value = getInput<std::string>("value");
  return value && !value.value().empty() ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
}

}  // namespace r2_bt
