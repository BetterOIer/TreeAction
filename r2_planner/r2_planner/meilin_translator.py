#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
meilin_translator.py - 梅林区动作序列翻译器

将外部规划器 mf_action_planner 发送的原始动作序列 (Float32MultiArray)
翻译为 BT 引擎消费的 segment JSON 格式。

输入: /mf_action_seq (std_msgs/Float32MultiArray)
输出: /planning/segments (std_msgs/String JSON)

每条动作 8 个元素: [arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7]
- arg0=0: move (arg1=row, arg2=col, arg3=height, arg4=yaw)
- arg0=1: fetch (arg1=row, arg2=col, arg3=height_diff, arg4=yaw)
"""

import json
import math
from typing import List, Dict, Any, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray


# 蓝区高度地图 (mm) - 根据 spec 定义
BLUE_HEIGHT_MAP: Dict[Tuple[int, int], int] = {
    (0, 0): 0, (0, 1): 0, (0, 2): 0,       # 入口区
    (1, 0): 400, (1, 1): 200, (1, 2): 400,
    (2, 0): 600, (2, 1): 400, (2, 2): 200,
    (3, 0): 400, (3, 1): 600, (3, 2): 400,  # 梅林区
    (4, 0): 200, (4, 1): 400, (4, 2): 200,
    (5, 0): 0, (5, 1): 0, (5, 2): 0,       # 出口区
}

# 红区高度地图 (仅 row=2 不同)
RED_HEIGHT_MAP: Dict[Tuple[int, int], int] = BLUE_HEIGHT_MAP.copy()
RED_HEIGHT_MAP.update({
    (2, 0): 200, (2, 1): 400, (2, 2): 600,  # 与蓝区相反
})


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-π, π] 范围"""
    return math.atan2(math.sin(angle), math.cos(angle))


class MeilinTranslator(Node):
    """梅林区动作序列翻译器节点"""

    def __init__(self):
        super().__init__('meilin_translator')

        # 声明参数
        self.declare_parameter("grid_size", 1.2)
        self.declare_parameter("grid_origin", [1.2, 1.2])  # [x0, y0]
        self.declare_parameter("grasp_distance", 0.4)
        self.declare_parameter("max_body_capacity", 3)
        self.declare_parameter("is_red_zone", False)
        self.declare_parameter("input_topic", "/mf_action_seq")
        self.declare_parameter("output_topic", "/planning/segments")

        # 加载参数
        self.grid_size: float = self.get_parameter("grid_size").value
        grid_origin = self.get_parameter("grid_origin").value
        self.grid_origin_x: float = grid_origin[0]
        self.grid_origin_y: float = grid_origin[1]
        self.grasp_distance: float = self.get_parameter("grasp_distance").value
        self.max_body_capacity: int = self.get_parameter("max_body_capacity").value
        self.is_red_zone: bool = self.get_parameter("is_red_zone").value
        input_topic: str = self.get_parameter("input_topic").value
        output_topic: str = self.get_parameter("output_topic").value

        self.get_logger().info(
            f"MeilinTranslator initialized: grid_size={self.grid_size}, "
            f"origin=({self.grid_origin_x}, {self.grid_origin_y}), "
            f"grasp_distance={self.grasp_distance}, "
            f"max_body_capacity={self.max_body_capacity}, "
            f"is_red_zone={self.is_red_zone}"
        )

        # Publisher: 匹配 BT 引擎的 QoS (reliable + transient_local)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.publisher = self.create_publisher(String, output_topic, qos)

        # Subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            input_topic,
            self.callback,
            10
        )

        # 状态追踪
        self.body_count: int = 0  # 车体中存储的 KFS 数量
        self.current_height: float = 0.0  # 当前平台高度 (mm)
        self.current_row: int = 0  # 当前所在行
        self.current_col: int = 0  # 当前所在列
        self.plan_counter: int = 0  # 计划 ID 计数器

    def callback(self, msg: Float32MultiArray) -> None:
        """处理接收到的动作序列"""
        data = msg.data

        # 验证输入长度 (必须是 8 的倍数)
        if len(data) % 8 != 0:
            self.get_logger().error(
                f"Invalid Float32MultiArray length: {len(data)}, not divisible by 8"
            )
            return

        if len(data) == 0:
            self.get_logger().warn("Empty Float32MultiArray received")
            return

        # 解析动作
        actions = self._parse_actions(data)
        if not actions:
            self.get_logger().error("Failed to parse actions")
            return

        # 翻译为 segments
        try:
            segments = self._translate_actions(actions)
        except Exception as e:
            self.get_logger().error(f"Translation failed: {e}")
            return

        # 发布 JSON
        self._publish_segments(segments)

    def _parse_actions(self, data: List[float]) -> List[Dict[str, Any]]:
        """解析 Float32MultiArray 为动作列表"""
        actions = []
        num_actions = len(data) // 8

        for i in range(num_actions):
            offset = i * 8
            action = {
                'arg0': int(data[offset + 0]),
                'arg1': data[offset + 1],  # row
                'arg2': data[offset + 2],  # col
                'arg3': data[offset + 3],  # height or height_diff
                'arg4': data[offset + 4],  # yaw
                'arg5': data[offset + 5],
                'arg6': data[offset + 6],
                'arg7': data[offset + 7],
            }
            actions.append(action)

        return actions

    def _translate_actions(self, actions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """将动作列表翻译为 segment 列表"""
        segments = []

        for action in actions:
            arg0 = action['arg0']
            row = int(action['arg1'])
            col = int(action['arg2'])
            arg3 = action['arg3']
            yaw = action['arg4']

            if arg0 == 0:
                # move 动作
                target_height = arg3
                segments.extend(self._translate_move(row, col, target_height, yaw))
            elif arg0 == 1:
                # fetch 动作
                height_diff = arg3
                segments.extend(self._translate_fetch(row, col, height_diff, yaw))
            else:
                self.get_logger().warn(f"Unknown action type: arg0={arg0}")

        return segments

    def _translate_move(
        self, row: int, col: int, target_height: float, yaw: float
    ) -> List[Dict[str, Any]]:
        """翻译 move 动作 → MOVE2 或 CLIMB"""
        segments = []

        # 计算高度变化
        height_diff = target_height - self.current_height

        if abs(height_diff) < 10:  # 无显著高度变化 → MOVE2
            x, y = self._grid_to_world(row, col)
            segments.append({
                "segment_type": "MOVE2",
                "move_target": {"x": x, "y": y, "yaw": yaw},
                "max_speed": 0.5,
                "timeout_sec": 30.0
            })
        else:  # 有高度变化 → CLIMB
            # 先移动到台阶边线
            climb_x, climb_y = self._calculate_climb_position(
                self.current_row, row, col
            )
            segments.append({
                "segment_type": "MOVE2",
                "move_target": {"x": climb_x, "y": climb_y, "yaw": yaw},
                "max_speed": 0.5,
                "timeout_sec": 30.0
            })
            # 执行攀爬
            segments.append({
                "segment_type": "CLIMB",
                "move_target": {"x": climb_x, "y": climb_y, "yaw": yaw},
                "climb_mode": "UP" if height_diff > 0 else "DOWN",
                "climb_direction": "FORWARD",
                "climb_height": abs(height_diff),
                "max_speed": 0.4,
                "timeout_sec": 30.0
            })

        # 更新状态
        self.current_height = target_height
        self.current_row = row
        self.current_col = col

        return segments

    def _translate_fetch(
        self, row: int, col: int, height_diff: float, yaw: float
    ) -> List[Dict[str, Any]]:
        """翻译 fetch 动作 → MOVE2 + GRASP + STORE"""
        segments = []

        # 1. 始终插入 MOVE2 到抓取位置
        grasp_pos = self._calculate_grasp_position(row, col, yaw)
        segments.append({
            "segment_type": "MOVE2",
            "move_target": grasp_pos,
            "max_speed": 0.5,
            "timeout_sec": 30.0
        })

        # 2. GRASP
        segments.append({
            "segment_type": "GRASP",
            "height_diff": height_diff,
            "timeout_sec": 30.0
        })

        # 3. STORE (根据 body_count 决定)
        if self.body_count < self.max_body_capacity:
            segments.append({
                "segment_type": "STORE1",
                "wait_result": False,
                "timeout_sec": 30.0
            })
            self.body_count += 1
        else:
            segments.append({
                "segment_type": "STORE2",
                "wait_result": False,
                "timeout_sec": 30.0
            })

        return segments

    def _grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """将网格坐标 (row, col) 转换为世界坐标 (x, y)"""
        x = self.grid_origin_x + row * self.grid_size
        y = self.grid_origin_y + col * self.grid_size
        return x, y

    def _calculate_grasp_position(
        self, row: int, col: int, yaw: float
    ) -> Dict[str, float]:
        """
        计算抓取位置。

        根据 spec 3.3:
        - 车身必须正对 KFS (yaw 指向格子中心)
        - 车停在靠近 KFS 的格子边线外一定距离
        """
        x_kfs, y_kfs = self._grid_to_world(row, col)
        half_grid = self.grid_size / 2  # 0.6m (边线到中心距离)
        d = self.grasp_distance

        # 归一化 yaw
        yaw = normalize_angle(yaw)

        # 根据朝向确定接近方向
        # yaw ≈ 0: 从 -Y 方向接近 (下方)
        # yaw ≈ π/-π: 从 +Y 方向接近 (上方)
        # yaw ≈ π/2: 从 -X 方向接近 (左侧)
        # yaw ≈ -π/2: 从 +X 方向接近 (右侧)

        tolerance = math.pi / 8  # 22.5° 容差

        if abs(yaw) < tolerance:
            # 从下方接近 (+X 方向朝向 KFS)
            x = x_kfs
            y = y_kfs - half_grid - d
        elif abs(yaw - math.pi) < tolerance or abs(yaw + math.pi) < tolerance:
            # 从上方接近 (-X 方向朝向 KFS)
            x = x_kfs
            y = y_kfs + half_grid + d
        elif abs(yaw - math.pi / 2) < tolerance:
            # 从左侧接近 (+Y 方向朝向 KFS)
            x = x_kfs - half_grid - d
            y = y_kfs
        else:
            # 从右侧接近 (-Y 方向朝向 KFS)
            x = x_kfs + half_grid + d
            y = y_kfs

        return {"x": x, "y": y, "yaw": yaw}

    def _calculate_climb_position(
        self, from_row: int, to_row: int, col: int
    ) -> Tuple[float, float]:
        """
        计算攀爬边线位置。

        根据 spec 3.4:
        - 车身应在台阶边线中间上下
        - 边线位置: x = x0 + (min_row + 0.5) × grid_size
        """
        edge_row = min(from_row, to_row) + 0.5
        x = self.grid_origin_x + edge_row * self.grid_size
        y = self.grid_origin_y + col * self.grid_size
        return x, y

    def _get_grid_height(self, row: int, col: int) -> int:
        """获取指定格子的高度 (mm)"""
        height_map = RED_HEIGHT_MAP if self.is_red_zone else BLUE_HEIGHT_MAP
        return height_map.get((row, col), 0)

    def _publish_segments(self, segments: List[Dict[str, Any]]) -> None:
        """发布 segment JSON"""
        self.plan_counter += 1
        plan_id = f"meilin_{self.plan_counter:03d}"

        plan = {
            "stage": "MEILIN",
            "plan_id": plan_id,
            "segments": segments
        }

        msg = String()
        msg.data = json.dumps(plan, ensure_ascii=False)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published plan: plan_id={plan_id}, segments={len(segments)}"
        )


def main(args: Optional[List[str]] = None) -> None:
    """节点入口函数"""
    rclpy.init(args=args)
    node = MeilinTranslator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
