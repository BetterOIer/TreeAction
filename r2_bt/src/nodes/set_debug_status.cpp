#include "r2_bt/nodes/debug/set_debug_status.hpp"

namespace r2_bt
{

SetDebugStatus::SetDebugStatus(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetDebugStatus::providedPorts()
{
  return {
    BT::InputPort<std::string>("execution_state", "Blackboard execution state to set"),
    BT::InputPort<std::string>("active_action", "Blackboard active action label to set"),
    BT::InputPort<std::string>("last_error", "Blackboard last error message to set"),
    BT::InputPort<int>("retry_count", "Blackboard retry count to set"),
  };
}

BT::NodeStatus SetDebugStatus::tick()
{
  if (auto value = getInput<std::string>("execution_state"))
  {
    config().blackboard->set("execution_state", value.value());
  }
  if (auto value = getInput<std::string>("active_action"))
  {
    config().blackboard->set("active_action", value.value());
  }
  if (auto value = getInput<std::string>("last_error"))
  {
    config().blackboard->set("last_error", value.value());
  }
  if (auto value = getInput<int>("retry_count"))
  {
    config().blackboard->set("retry_count", value.value());
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt
