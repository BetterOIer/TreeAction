#pragma once

#include <behaviortree_cpp/action_node.h>
#include <r2_interfaces/action/suspension_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mutex>
#include <string>

namespace r2_bt
{

class SuspensionControl : public BT::StatefulActionNode
{
public:
  SuspensionControl(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SuspensionAction = r2_interfaces::action::SuspensionControl;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SuspensionAction>;

  rclcpp_action::Client<SuspensionAction>::SharedPtr action_client_;
  std::shared_ptr<GoalHandle> goal_handle_;
  rclcpp::Node::SharedPtr node_;

  bool goal_accepted_ = false;
  bool goal_done_ = false;
  BT::NodeStatus result_status_ = BT::NodeStatus::FAILURE;
  std::string result_message_;
  int final_state_ = 0;
  std::mutex mutex_;
};

}  // namespace r2_bt
