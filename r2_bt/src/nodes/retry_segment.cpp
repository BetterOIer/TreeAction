#include "r2_bt/nodes/decorators/retry_segment.hpp"

namespace r2_bt
{

RetrySegment::RetrySegment(const std::string& name, const BT::NodeConfig& config)
  : BT::DecoratorNode(name, config)
{
}

BT::PortsList RetrySegment::providedPorts()
{
  return {
    BT::InputPort<int>("num_attempts", "Maximum attempts, including the first one"),
  };
}

BT::NodeStatus RetrySegment::tick()
{
  const int max_attempts = getInput<int>("num_attempts").value_or(3);
  config().blackboard->set("retry_count", try_count_);

  const auto child_status = child_node_->executeTick();
  if (child_status == BT::NodeStatus::RUNNING)
  {
    return BT::NodeStatus::RUNNING;
  }

  if (child_status == BT::NodeStatus::SUCCESS)
  {
    try_count_ = 0;
    config().blackboard->set("retry_count", 0);
    resetChild();
    return BT::NodeStatus::SUCCESS;
  }

  if (child_status == BT::NodeStatus::FAILURE)
  {
    try_count_++;
    config().blackboard->set("retry_count", try_count_);
    resetChild();

    if (max_attempts < 0 || try_count_ < max_attempts)
    {
      config().blackboard->set("execution_state", std::string{"ACTION_RETRYING"});
      return BT::NodeStatus::RUNNING;
    }

    try_count_ = 0;
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

void RetrySegment::halt()
{
  try_count_ = 0;
  config().blackboard->set("retry_count", 0);
  BT::DecoratorNode::halt();
}

}  // namespace r2_bt
