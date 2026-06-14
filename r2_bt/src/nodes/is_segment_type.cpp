#include "r2_bt/nodes/conditions/is_segment_type.hpp"

namespace r2_bt
{

IsSegmentType::IsSegmentType(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList IsSegmentType::providedPorts()
{
  return {
    BT::InputPort<std::string>("expected", "Expected segment type string (e.g. MOVE2, CLIMB)"),
    BT::InputPort<std::string>("actual", "Actual segment type from blackboard (e.g. {segment_type})"),
  };
}

BT::NodeStatus IsSegmentType::tick()
{
  auto expected = getInput<std::string>("expected");
  auto actual = getInput<std::string>("actual");
  if (!expected || !actual)
  {
    return BT::NodeStatus::FAILURE;
  }
  return expected.value() == actual.value() ? BT::NodeStatus::SUCCESS
                                            : BT::NodeStatus::FAILURE;
}

}  // namespace r2_bt
