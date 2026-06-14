#pragma once

#include <behaviortree_cpp/actions/pop_from_queue.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace r2_bt
{

struct Segment
{
  int index = 0;
  std::string segment_type;
  std::string debug_name;

  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;
  double max_speed = 0.5;
  double timeout_sec = 30.0;

  int climb_mode = 0;
  int climb_direction = 0;
  double climb_height = 0.0;

  uint8_t arm_command = 0;  // uint8 常量，对应 ArmAction.action 中的 CMD_*
  bool wait_result = true;

  uint8_t spear_command = 0;  // uint8 常量，对应 SpearAction.action 中的 CMD_*
};

using SegmentQueue = BT::ProtectedQueue<Segment>;
using SegmentQueuePtr = std::shared_ptr<SegmentQueue>;

struct ArmRuntimeState
{
  std::mutex mtx;
  bool background_active = false;
  bool background_done = true;
  bool background_success = true;
  std::string background_message;
  std::string current_command;
};

using ArmRuntimeStatePtr = std::shared_ptr<ArmRuntimeState>;

}  // namespace r2_bt
