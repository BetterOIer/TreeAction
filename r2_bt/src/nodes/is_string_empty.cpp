#include "r2_bt/nodes/conditions/is_string_empty.hpp"

namespace r2_bt
{

IsStringEmpty::IsStringEmpty(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList IsStringEmpty::providedPorts()
{
  return {
    BT::InputPort<std::string>("value", "String to test — SUCCESS if empty or missing"),
  };
}

BT::NodeStatus IsStringEmpty::tick()
{
  auto value = getInput<std::string>("value");
  return !value || value.value().empty() ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
}

}  // namespace r2_bt
