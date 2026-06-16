#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "r2_bt/nodes/actions/arm_action.hpp"
#include "r2_bt/nodes/actions/move_to_pose.hpp"
#include "r2_bt/nodes/actions/pop_next_segment.hpp"
#include "r2_bt/nodes/actions/spear_action.hpp"
#include "r2_bt/nodes/actions/suspension_control.hpp"
#include "r2_bt/nodes/actions/wait_arm_idle.hpp"
#include "r2_bt/nodes/conditions/is_segment_type.hpp"
#include "r2_bt/nodes/conditions/is_string_empty.hpp"
#include "r2_bt/nodes/conditions/is_string_non_empty.hpp"
#include "r2_bt/nodes/conditions/wait_for_r1_signal.hpp"
#include "r2_bt/nodes/decorators/retry_segment.hpp"
#include "r2_bt/nodes/debug/set_debug_status.hpp"

class BtEngineNode : public rclcpp::Node
{
public:
  BtEngineNode() : Node("bt_engine_node")
  {
    declare_parameter("groot2_port", 1667);
    declare_parameter("tick_frequency", 100.0);
    declare_parameter("segment_topic", "/planning/segments");
    declare_parameter("tree_file", "full_match.xml");

    groot2_port_ = static_cast<unsigned>(get_parameter("groot2_port").as_int());
    double tick_freq = get_parameter("tick_frequency").as_double();
    segment_topic_ = get_parameter("segment_topic").as_string();
    tree_file_ = get_parameter("tree_file").as_string();

    factory_.registerNodeType<r2_bt::MoveToPose>("MoveToPose");
    factory_.registerNodeType<r2_bt::PopNextSegment>("PopNextSegment");
    factory_.registerNodeType<r2_bt::SuspensionControl>("SuspensionControl");
    factory_.registerNodeType<r2_bt::SpearAction>("SpearAction");
    factory_.registerNodeType<r2_bt::ArmAction>("ArmAction");
    factory_.registerNodeType<r2_bt::WaitArmIdle>("WaitArmIdle");
    factory_.registerNodeType<r2_bt::IsSegmentType>("IsSegmentType");
    factory_.registerNodeType<r2_bt::IsStringEmpty>("IsStringEmpty");
    factory_.registerNodeType<r2_bt::IsStringNonEmpty>("IsStringNonEmpty");
    factory_.registerNodeType<r2_bt::WaitForR1Signal>("WaitForR1Signal");
    factory_.registerNodeType<r2_bt::RetrySegment>("RetrySegment");
    factory_.registerNodeType<r2_bt::SetDebugStatus>("SetDebugStatus");

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("segment_queue", std::make_shared<r2_bt::SegmentQueue>());
    blackboard_->set("arm_state", std::make_shared<r2_bt::ArmRuntimeState>());
    blackboard_->set("segment_json", std::string{});
    blackboard_->set("plan_id", std::string{});
    blackboard_->set("current_segment_index", -1);
    blackboard_->set("segment_type", std::string{});
    blackboard_->set("segment_debug_name", std::string{});
    blackboard_->set("active_action", std::string{});
    blackboard_->set("retry_count", 0);
    blackboard_->set("last_error", std::string{});
    blackboard_->set("execution_state", std::string{"WAITING_PLAN"});

    segment_sub_ = create_subscription<std_msgs::msg::String>(
        segment_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&BtEngineNode::segment_callback, this, std::placeholders::_1));

    build_fixed_tree();

    auto tick_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tick_freq));
    tick_timer_ = create_wall_timer(tick_period,
                                    std::bind(&BtEngineNode::tick_callback, this));

    RCLCPP_INFO(get_logger(),
                "BT Engine started: tick=%.1fHz, groot2_port=%u, segment_topic=%s, tree_file=%s",
                tick_freq, groot2_port_, segment_topic_.c_str(), tree_file_.c_str());
  }

  ~BtEngineNode() override
  {
    groot2_publisher_.reset();
    if (current_tree_)
    {
      current_tree_->haltTree();
    }
  }

private:
  void segment_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string plan_id;
    auto segments = parse_segment_json(msg->data, plan_id);
    if (segments.empty())
    {
      RCLCPP_ERROR(get_logger(), "Segment plan rejected: no executable segments");
      blackboard_->set("last_error", std::string{"Segment plan rejected"});
      blackboard_->set("execution_state", std::string{"MISSION_FAILED"});
      return;
    }

    auto queue = std::make_shared<r2_bt::SegmentQueue>();
    {
      std::unique_lock<std::mutex> lock(queue->mtx);
      queue->items.assign(segments.begin(), segments.end());
    }

    blackboard_->set("segment_json", msg->data);
    blackboard_->set("segment_queue", queue);
    blackboard_->set("plan_id", plan_id);
    blackboard_->set("current_segment_index", -1);
    blackboard_->set("segment_type", std::string{});
    blackboard_->set("segment_debug_name", std::string{});
    blackboard_->set("active_action", std::string{});
    blackboard_->set("retry_count", 0);
    blackboard_->set("last_error", std::string{});
    blackboard_->set("execution_state", std::string{"WAITING_PLAN"});

    RCLCPP_INFO(get_logger(), "Segment plan received: plan_id=%s, segments=%zu",
                plan_id.c_str(), segments.size());
  }

  int parse_climb_mode(const std::string& value) const
  {
    if (value == "UP")
    {
      return 1;
    }
    if (value == "DOWN")
    {
      return 2;
    }
    if (value == "RECOVER")
    {
      return 3;
    }
    return 0;
  }

  int parse_climb_direction(const std::string& value) const
  {
    if (value == "LEFT")
    {
      return 1;
    }
    if (value == "RIGHT")
    {
      return 2;
    }
    if (value == "BACKWARD")
    {
      return 3;
    }
    return 0;
  }

  uint8_t parse_arm_command(const std::string& value) const
  {
    if (value == "grasp") return 1;
    if (value == "store_to_body") return 2;
    if (value == "store_on_arm") return 3;
    if (value == "get_body") return 4;
    if (value == "place_mid") return 5;
    if (value == "place_high") return 6;
    if (value == "safe_pose") return 7;
    if (value == "prepare_grasp") return 8;
    return 0;
  }

  uint8_t parse_spear_command(const std::string& value) const
  {
    if (value == "prepare") return 1;
    if (value == "grasp") return 2;
    if (value == "dock_extend") return 3;
    if (value == "dock_release") return 4;
    return 0;
  }

  void load_move_target(const nlohmann::json& item, r2_bt::Segment& segment)
  {
    if (!item.contains("move_target") || !item["move_target"].is_object())
    {
      throw std::runtime_error(segment.segment_type + " requires move_target");
    }
    const auto& target = item["move_target"];
    segment.target_x = target.at("x").get<double>();
    segment.target_y = target.at("y").get<double>();
    segment.target_yaw = target.value("yaw", 0.0);
  }

  std::vector<r2_bt::Segment> parse_segment_json(const std::string& json_str,
                                                 std::string& plan_id)
  {
    std::vector<r2_bt::Segment> result;

    try
    {
      const auto doc = nlohmann::json::parse(json_str);
      if (!doc.contains("segments") || !doc["segments"].is_array())
      {
        throw std::runtime_error("top-level segments array is required");
      }

      plan_id = doc.value("plan_id", "");
      const auto stage = doc.value("stage", "");
      if (!stage.empty() && stage != "FULL_MATCH" && stage != "PREPARE" &&
          stage != "MEILIN" && stage != "FINAL")
      {
        RCLCPP_WARN(get_logger(), "Unexpected segment stage: %s", stage.c_str());
      }

      int index = 0;
      for (const auto& item : doc["segments"])
      {
        if (!item.is_object())
        {
          throw std::runtime_error("segment item must be an object");
        }

        r2_bt::Segment segment;
        segment.index = index++;
        segment.segment_type = item.at("segment_type").get<std::string>();
        segment.debug_name = segment.segment_type + "#" + std::to_string(segment.index);

        if (segment.segment_type == "SPEAR_PREP")
        {
          // SpearAction(prepare) + Align
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.spear_command = 1;  // CMD_PREPARE
        }
        else if (segment.segment_type == "SPEAR_GRASP")
        {
          segment.timeout_sec = item.value("timeout_sec", 5.0);
          segment.spear_command = 2;  // CMD_GRASP
        }
        else if (segment.segment_type == "ALIGN")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
        }
        else if (segment.segment_type == "DOCK")
        {
          segment.timeout_sec = item.value("timeout_sec", 10.0);
          // DOCK segment: BT XML uses inline dock_extend/dock_release commands,
          // spear_command from JSON is not used for DOCK.
        }
        else if (segment.segment_type == "MOVE2")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.5);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
        }
        else if (segment.segment_type == "GRASP")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 1;  // CMD_GRASP
          segment.wait_result = true;
          segment.height_diff = item.value("height_diff", 0.0);
        }
        else if (segment.segment_type == "CLIMB")
        {
          load_move_target(item, segment);
          segment.climb_mode = parse_climb_mode(item.at("climb_mode").get<std::string>());
          segment.climb_direction =
            parse_climb_direction(item.at("climb_direction").get<std::string>());
          segment.climb_height = item.at("climb_height").get<double>();
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = parse_arm_command(item.value("arm_command", ""));
          segment.wait_result = item.value("wait_result", true);
        }
        else if (segment.segment_type == "STORE1")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 2;  // CMD_STORE_TO_BODY
          segment.wait_result = item.value("wait_result", false);
        }
        else if (segment.segment_type == "STORE2")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 3;  // CMD_STORE_ON_ARM
          segment.wait_result = item.value("wait_result", false);
        }
        else if (segment.segment_type == "EXIT")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.6);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 7;  // CMD_SAFE_POSE
          segment.wait_result = true;
        }
        else if (segment.segment_type == "FINAL_MOVE2")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
        }
        else if (segment.segment_type == "PLACE_MID")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 5;  // CMD_PLACE_MID
          segment.wait_result = true;
        }
        else if (segment.segment_type == "PLACE_HIGH")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 6;  // CMD_PLACE_HIGH
          segment.wait_result = true;
        }
        else if (segment.segment_type == "FINISH")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 7;  // CMD_SAFE_POSE
          segment.wait_result = true;
        }
        else
        {
          throw std::runtime_error("unsupported segment_type: " + segment.segment_type);
        }

        result.push_back(segment);
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse segment JSON: %s", e.what());
      result.clear();
    }

    return result;
  }

  std::string resolve_tree_file() const
  {
    if (!tree_file_.empty() && tree_file_.front() == '/')
    {
      return tree_file_;
    }

    auto share_dir = ament_index_cpp::get_package_share_directory("r2_bt");
    return share_dir + "/trees/" + tree_file_;
  }

  void build_fixed_tree()
  {
    groot2_publisher_.reset();
    if (current_tree_)
    {
      current_tree_->haltTree();
      current_tree_.reset();
    }

    auto tree_path = resolve_tree_file();
    RCLCPP_INFO(get_logger(), "Loading fixed behavior tree: %s", tree_path.c_str());

    try
    {
      current_tree_ = std::make_unique<BT::Tree>(
          factory_.createTreeFromFile(tree_path, blackboard_));

      try
      {
        groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(
            *current_tree_, groot2_port_);
        RCLCPP_INFO(get_logger(), "Groot2 publisher started on port %u", groot2_port_);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(get_logger(), "Fixed tree loaded, but Groot2 publisher failed: %s",
                    e.what());
      }

      RCLCPP_INFO(get_logger(), "Fixed tree loaded successfully");
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load fixed tree: %s", e.what());
      current_tree_.reset();
    }
  }

  void tick_callback()
  {
    if (current_tree_)
    {
      blackboard_->set("ros_node", shared_from_this());
      auto status = current_tree_->tickOnce();
      if (status == BT::NodeStatus::FAILURE)
      {
        blackboard_->set("execution_state", std::string{"MISSION_FAILED"});
      }
    }
  }

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr segment_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  unsigned groot2_port_ = 1667;
  std::string segment_topic_;
  std::string tree_file_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtEngineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
