#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <mutex>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

/**
 * @brief 等待后台机械臂动作完成
 *
 * 订阅 /arm_runtime_state (std_msgs/Bool)，当收到 true 时表示后台动作完成。
 * 这是连接 C++ BT 层和 Python Action Server 的桥梁。
 */
class WaitArmIdle : public BT::StatefulActionNode
{
public:
  WaitArmIdle(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void state_callback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::chrono::steady_clock::time_point start_time_;
  bool state_received_ = false;
  bool last_state_ = false;
};

}  // namespace r2_bt
