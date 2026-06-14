#include "r2_bt/nodes/actions/arm_action.hpp"

#include <cmath>
#include <cstdint>

namespace r2_bt
{

ArmAction::ArmAction(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList ArmAction::providedPorts()
{
  return {
    BT::InputPort<uint8_t>("command"),
    BT::InputPort<bool>("wait_result", true, "Return only after action result"),
    BT::InputPort<double>("timeout_sec", 30.0, "Abort action after this many seconds"),
    BT::InputPort<std::string>("server_name", "arm_action", "ROS 2 action name"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus ArmAction::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    goal_response_received_ = false;
    goal_accepted_ = false;
    goal_done_ = false;
    keep_background_goal_ = false;
    result_status_ = BT::NodeStatus::FAILURE;
    result_message_.clear();
  }

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  arm_state_ = config().blackboard->get<ArmRuntimeStatePtr>("arm_state");
  if (!node_ || !arm_state_)
  {
    result_message_ = "Missing ros_node or arm_state on blackboard";
    setOutput("message", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  const auto command = getInput<uint8_t>("command");
  if (!command)
  {
    result_message_ = "Missing ArmAction command";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  wait_result_ = getInput<bool>("wait_result").value_or(true);
  timeout_sec_ = getInput<double>("timeout_sec").value_or(30.0);
  const auto server_name = getInput<std::string>("server_name").value_or("arm_action");

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(node_, server_name);
  }

  if (!action_client_->action_server_is_ready())
  {
    result_message_ = "ArmAction action server not available";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    RCLCPP_ERROR(node_->get_logger(), "[ArmAction] %s", result_message_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal = ActionT::Goal();
  goal.command = command.value();
  goal.wait_result = wait_result_;
  goal.timeout_sec = static_cast<float>(timeout_sec_);

  {
    std::lock_guard<std::mutex> lock(arm_state_->mtx);
    arm_state_->current_command = goal.command;
    if (!wait_result_)
    {
      arm_state_->background_active = true;
      arm_state_->background_done = false;
      arm_state_->background_success = false;
      arm_state_->background_message.clear();
    }
  }

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
        result_message_ = "Goal rejected by arm action server";
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

      if (arm_state_ && keep_background_goal_)
      {
        std::lock_guard<std::mutex> arm_lock(arm_state_->mtx);
        arm_state_->background_done = true;
        arm_state_->background_success = result_status_ == BT::NodeStatus::SUCCESS;
        arm_state_->background_message = result_message_;
      }
    };

  start_time_ = std::chrono::steady_clock::now();
  keep_background_goal_ = !wait_result_;
  config().blackboard->set("active_action", std::string{"ArmAction:"} + std::to_string(static_cast<int>(goal.command)));
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  action_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(), "[ArmAction] Goal sent: command=%d wait_result=%s",
              static_cast<int>(goal.command), wait_result_ ? "true" : "false");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmAction::onRunning()
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
    if (arm_state_ && keep_background_goal_)
    {
      std::lock_guard<std::mutex> arm_lock(arm_state_->mtx);
      arm_state_->background_done = true;
      arm_state_->background_success = false;
      arm_state_->background_message = result_message_;
    }
    return BT::NodeStatus::FAILURE;
  }

  if (!wait_result_)
  {
    config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
    return BT::NodeStatus::SUCCESS;
  }

  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (timeout_sec_ > 0.0 && elapsed > timeout_sec_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
    result_message_ = "ArmAction timed out";
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

void ArmAction::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!keep_background_goal_ && action_client_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[ArmAction] Halted");
  }
}

}  // namespace r2_bt