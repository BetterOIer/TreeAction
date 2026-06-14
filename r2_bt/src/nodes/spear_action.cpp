#include "r2_bt/nodes/actions/spear_action.hpp"

#include <cstdint>

namespace r2_bt
{

SpearAction::SpearAction(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList SpearAction::providedPorts()
{
  return {
    BT::InputPort<uint8_t>("command"),
    BT::InputPort<double>("timeout_sec", 5.0, "Abort action after this many seconds"),
    BT::InputPort<std::string>("server_name", "spear_action", "ROS 2 action name"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus SpearAction::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    goal_response_received_ = false;
    goal_accepted_ = false;
    goal_done_ = false;
    result_status_ = BT::NodeStatus::FAILURE;
    result_message_.clear();
  }

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  if (!node_)
  {
    result_message_ = "Missing ros_node on blackboard";
    setOutput("message", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  const auto command = getInput<uint8_t>("command");
  if (!command)
  {
    result_message_ = "Missing SpearAction command";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  timeout_sec_ = getInput<double>("timeout_sec").value_or(5.0);
  const auto server_name = getInput<std::string>("server_name").value_or("spear_action");

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(node_, server_name);
  }

  if (!action_client_->action_server_is_ready())
  {
    result_message_ = "SpearAction action server not available";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    RCLCPP_ERROR(node_->get_logger(), "[SpearAction] %s", result_message_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal = ActionT::Goal();
  goal.command = command.value();
  goal.timeout_sec = static_cast<float>(timeout_sec_);

  auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const std::shared_ptr<GoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_response_received_ = true;
      goal_handle_ = goal_handle;
      goal_accepted_ = static_cast<bool>(goal_handle);
      if (!goal_handle)
      {
        goal_done_ = true;
        result_status_ = BT::NodeStatus::FAILURE;
        result_message_ = "Goal rejected by spear action server";
      }
    };
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_done_ = true;
      if (result.result)
      {
        result_message_ = result.result->message;
      }
      result_status_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                        result.result && result.result->success)
                         ? BT::NodeStatus::SUCCESS
                         : BT::NodeStatus::FAILURE;
    };

  start_time_ = std::chrono::steady_clock::now();
  config().blackboard->set("active_action", std::string{"SpearAction:"} + std::to_string(static_cast<int>(goal.command)));
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  action_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(), "[SpearAction] Goal sent: command=%d timeout=%.1f",
              static_cast<int>(goal.command), goal.timeout_sec);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpearAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!goal_response_received_)
  {
    return BT::NodeStatus::RUNNING;
  }

  if (!goal_accepted_)
  {
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (timeout_sec_ > 0.0 && elapsed > timeout_sec_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
    result_message_ = "SpearAction timed out";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_done_)
  {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("message", result_message_);
  config().blackboard->set(
    "execution_state",
    result_status_ == BT::NodeStatus::SUCCESS ? std::string{"ACTION_SUCCESS"}
                                              : std::string{"ACTION_FAILED"});
  if (result_status_ == BT::NodeStatus::FAILURE)
  {
    config().blackboard->set("last_error", result_message_);
  }
  return result_status_;
}

void SpearAction::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (action_client_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[SpearAction] Halted");
  }
}

}  // namespace r2_bt
