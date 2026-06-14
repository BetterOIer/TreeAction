# R2 Autonomy Core

RoboCon 2026 "武林探秘" R2 全自主决策与执行中间件。

本仓库负责把外部全流程规划结果转换为机器人语义动作：接收 `/planning/segments` 的 JSON，维护 `segment_queue`，由固定 BehaviorTree.CPP 顶层树和 3 个子树逐段调用底盘、悬挂、机械臂和矛头机构 Action。本仓库不负责路径规划、KFS 选择、入口/出口选择、路线合法性判断或 R1/R2 无线通信。

## 架构

```text
外部规划模块
  └── /planning/segments (std_msgs/String JSON)
        ↓
r2_bt
  ├── 固定 BT XML: full_match.xml
  │     ├── PrepareArea  (矛头抓取、对接)
  │     ├── MeilinArea   (KFS 拿取、搬运、放置)
  │     └── FinalArea    (中层/上层 KFS 放置)
  ├── segment_queue / PopNextSegment
  ├── Groot2 调试黑板
  └── Action Clients
        ↓
r2_hardware (Action Server 层)
  ├── move_to_pose          (底盘导航/精调)
  ├── suspension_control    (四轮独立悬挂)
  ├── arm_action            (机械臂语义控制)
  └── spear_action          (矛头机构控制)
        ↓
ares_usb / sensor nodes / 下位机
```

### 包职责

| 包 | 职责 |
|----|------|
| `r2_bt` | BT 决策层；解析完整比赛 segment 计划，按固定树和 3 个子树调度语义 Action，维护 Groot2 调试字段 |
| `r2_interfaces` | 跨包 Action 接口定义：`MoveToPose`、`SuspensionControl`、`ArmAction`、`SpearAction` |
| `r2_hardware` | 硬件执行层：各 Action Server 实现（底盘、悬挂、机械臂、矛头机构）及传感器驱动 |
| `ares_usb` | ROS 2 与下位机 USB 透传 |
| `r2_bringup` | 仿真/实车启动文件 |

## 当前 Segment 架构

BT 引擎默认订阅：

```text
/planning/segments
```

消息类型：

```text
std_msgs/msg/String
```

内容是 JSON，顶层字段：

| 字段 | 说明 |
|------|------|
| `stage` | 当前阶段；完整比赛使用 `FULL_MATCH`，也支持 `PREPARE`、`MEILIN`、`FINAL` 调试 |
| `plan_id` | 可选，调试和日志标识 |
| `segments` | 必填，按执行顺序排列的 segment 数组 |

支持的 `segment_type`：

```text
SPEAR_PREP
SPEAR_GRASP
ALIGN
DOCK
MOVE2
GRASP
CLIMB
STORE1
STORE2
EXIT
FINAL_MOVE2
PLACE_MID
PLACE_HIGH
FINISH
```

BT 不重排、不重规划、不判断路线是否合法，只做字段校验、入队和执行调度。

## 三个子树

当前默认树：`r2_bt/trees/full_match.xml`

| 子树 | 覆盖流程 | segment_type |
|------|----------|--------------|
| `PrepareArea` | 准备区拿取矛头、精调对齐、与 R1 对接 | `SPEAR_PREP`, `SPEAR_GRASP`, `ALIGN`, `DOCK` |
| `MeilinArea` | 梅林区格间移动、上下台阶、抓取和转存 KFS、离开梅林 | `MOVE2`, `GRASP`, `CLIMB`, `STORE1`, `STORE2`, `EXIT` |
| `FinalArea` | 竞技区中层放置 KFS、与 R1 合体放上层、安全收尾 | `FINAL_MOVE2`, `PLACE_MID`, `PLACE_HIGH`, `FINISH` |

## Segment 字段

### 准备区

| segment_type | 必填字段 | 默认/行为 |
|--------------|----------|-----------|
| `SPEAR_PREP` | `move_target` | `spear_command="prepare"` + `MoveToPose`（低速精调），`max_speed=0.4`，硬同步点 |
| `SPEAR_GRASP` | 无 | `SpearAction(command=grasp)`，`timeout_sec=5.0`，硬同步点 |
| `ALIGN` | `move_target` | `MoveToPose`（低速精调），`max_speed=0.4` |
| `DOCK` | 无 | `SpearAction(dock_extend)` → 等待 R1 信号(待实现) → `SpearAction(dock_release)`，`timeout_sec=10.0` |

### 梅林区

| segment_type | 必填字段 | 默认/行为 |
|--------------|----------|-----------|
| `MOVE2` | `move_target` | `max_speed=0.5`，调用 `MoveToPose`（格间精调） |
| `GRASP` | 无 | `ArmAction(command=grasp)`，硬同步点 |
| `CLIMB` | `climb_mode`, `climb_direction`, `climb_height`, `move_target` | 并行 `SuspensionControl` + `MoveToPose`；可选并行 `arm_command` |
| `STORE1` | 无 | `ArmAction(command=store_to_body)`，默认 `wait_result=false` |
| `STORE2` | 无 | `ArmAction(command=store_on_arm)`，默认 `wait_result=false` |
| `EXIT` | `move_target` | `ArmAction(command=safe_pose)`，悬挂恢复，再 `MoveToPose` |

### 竞技区

| segment_type | 必填字段 | 默认/行为 |
|--------------|----------|-----------|
| `FINAL_MOVE2` | `move_target` | `max_speed=0.4`，调用 `MoveToPose`（区域间导航） |
| `PLACE_MID` | 无 | `ArmAction(command=place_mid)` |
| `PLACE_HIGH` | 无 | `ArmAction(command=place_high)` |
| `FINISH` | 无 | `ArmAction(command=safe_pose)`，悬挂恢复 |

坐标约定：

| 字段 | 坐标系/单位 |
|------|-------------|
| `move_target.x/y` | 世界坐标，m |
| `move_target.yaw` | 世界坐标朝向，rad |
| `kfs_pose.x/y/z` | 机械臂或车体局部坐标，m |
| `kfs_pose.yaw` | rad |
| `climb_height` | mm |
| `max_speed` | m/s |
| `timeout_sec` | s |

枚举：

| 字段 | 可选值 |
|------|--------|
| `climb_mode` | `AUTO`, `UP`, `DOWN`, `RECOVER` |
| `climb_direction` | `FORWARD`, `LEFT`, `RIGHT`, `BACKWARD` |
| `SuspensionControl.mode` | `0=AUTO`, `1=CLIMB_UP`, `2=CLIMB_DOWN`, `3=RECOVER` |
| `SpearAction.command` | `prepare`, `grasp`, `dock_extend`, `dock_release` |
| `ArmAction.command` | `grasp`, `store_to_body`, `store_on_arm`, `get_body`, `place_mid`, `place_high`, `safe_pose`, `idle` |

### 移动类型区分

| Action | 类型 | 用途 | 特点 |
|--------|------|------|------|
| `MoveToPose` | 导航 | 区域间切换 | 长距离、路径规划、常规速度 |
| `Align` (待实现) | 精调 | 区域内小段直线运动 | 短距离、直线轨迹、低速高精度 |

当前 `Align` 由 `MoveToPose` 以低速档（`max_speed=0.4`）代替，后续需实现独立精调 Action。

## JSON 示例

```json
{
  "stage": "FULL_MATCH",
  "plan_id": "full_demo_001",
  "segments": [
    {
      "segment_type": "SPEAR_PREP",
      "move_target": {"x": 0.4, "y": 0.0, "yaw": 0.0},
      "max_speed": 0.4,
      "timeout_sec": 30.0
    },
    {
      "segment_type": "SPEAR_GRASP",
      "timeout_sec": 5.0
    },
    {
      "segment_type": "ALIGN",
      "move_target": {"x": 0.8, "y": 0.0, "yaw": 0.0},
      "max_speed": 0.4,
      "timeout_sec": 30.0
    },
    {
      "segment_type": "DOCK",
      "timeout_sec": 10.0
    },
    {
      "segment_type": "MOVE2",
      "move_target": {"x": 1.2, "y": 0.8, "yaw": 1.57},
      "max_speed": 0.5,
      "timeout_sec": 20.0
    },
    {
      "segment_type": "GRASP",
      "kfs_id": "KFS_A"
    },
    {
      "segment_type": "STORE1",
      "wait_result": false
    },
    {
      "segment_type": "MOVE2",
      "move_target": {"x": 1.8, "y": 0.8, "yaw": 0.0}
    },
    {
      "segment_type": "CLIMB",
      "climb_mode": "UP",
      "climb_direction": "FORWARD",
      "climb_height": 200,
      "move_target": {"x": 2.2, "y": 0.8, "yaw": 0.0},
      "max_speed": 0.4,
      "timeout_sec": 30.0,
      "arm_command": "store_to_body",
      "wait_result": true
    },
    {
      "segment_type": "EXIT",
      "move_target": {"x": 3.0, "y": 0.8, "yaw": 0.0}
    },
    {
      "segment_type": "FINAL_MOVE2",
      "move_target": {"x": 3.4, "y": 1.0, "yaw": 0.0}
    },
    {
      "segment_type": "PLACE_MID",
      "kfs_id": "KFS_A"
    },
    {
      "segment_type": "PLACE_HIGH",
      "kfs_id": "KFS_A"
    },
    {
      "segment_type": "FINISH"
    }
  ]
}
```

快速发布示例：

```bash
ros2 topic pub --once /planning/segments std_msgs/msg/String "{data: '{\"stage\":\"MEILIN\",\"plan_id\":\"demo_001\",\"segments\":[{\"segment_type\":\"MOVE2\",\"move_target\":{\"x\":1.0,\"y\":0.0,\"yaw\":0.0}}]}'}"
```

## BT 节点

当前默认树：`r2_bt/trees/full_match.xml`

| 节点 | 类型 | 作用 |
|------|------|------|
| `PopNextSegment` | `StatefulActionNode` | 等待并弹出下一个 segment，写入黑板 |
| `IsSegmentType` | `ConditionNode` | 判断当前 segment 类型 |
| `IsStringEmpty` / `IsStringNonEmpty` | `ConditionNode` | 判断可选字段是否存在 |
| `WaitArmIdle` | `StatefulActionNode` | 等后台机械臂动作结束，并检查结果 |
| `MoveToPose` | `StatefulActionNode` | 异步调用 `move_to_pose` |
| `SuspensionControl` | `StatefulActionNode` | 异步调用 `suspension_control` |
| `ArmAction` | `StatefulActionNode` | 异步调用 `arm_action` |
| `SpearAction` | `StatefulActionNode` | 异步调用 `spear_action` |
| `RetrySegment` | `DecoratorNode` | segment 失败重试，并维护 `retry_count` |
| `SetDebugStatus` | `SyncActionNode` | 写入 Groot2 可见的调试字段 |

关键行为：

- `MoveToPose`、`SuspensionControl`、`ArmAction`、`SpearAction` 都由 BT 节点异步发送 goal。
- `onHalted()` 会取消正在执行的 Action；后台机械臂动作除外，它由 `WaitArmIdle` 汇合。
- 准备区、梅林区和竞技区关键 segment 使用 `RetrySegment num_attempts="3"`，即失败后最多重试两次。
- `CLIMB` 三次失败后会尝试 `SuspensionControl(mode=RECOVER)`，然后整段失败。

## Groot2 调试字段

黑板维护以下字段，Groot2 连接后可观察：

```text
plan_id
current_segment_index
segment_type
segment_debug_name
active_action
retry_count
last_error
execution_state
```

`execution_state` 取值：

```text
WAITING_PLAN
SEGMENT_LOADED
ACTION_RUNNING
ACTION_SUCCESS
ACTION_FAILED
MISSION_SUCCESS
MISSION_FAILED
```

连接方式：

```text
Groot2 -> localhost:1667
```

## Action 接口

| Action | 默认服务名 | 说明 |
|--------|------------|------|
| `MoveToPose` | `move_to_pose` | 底盘运动到目标位姿（区域间导航 / 区域内精调） |
| `SuspensionControl` | `suspension_control` | 四轮独立悬挂上下台阶、恢复行驶高度 |
| `ArmAction` | `arm_action` | 机械臂语义控制：抓取、转存、放置、安全位，支持后台执行 |
| `SpearAction` | `spear_action` | 矛头机构控制：准备、抓取、伸出、释放 |

所有 Action 均使用 ROS 2 Action 协议（`rclcpp_action` / `rclpy.action`），BT 层通过 `StatefulActionNode` 异步调用，硬件层通过 `ActionServer` 执行。

### Action 实现状态

| Action | .action 定义 | BT 节点 (C++) | Action Server (Python) | 硬件 topic |
|--------|:--:|:--:|:--:|------|
| `MoveToPose` | ✅ | ✅ | ✅ | `/t0x0101` |
| `SuspensionControl` | ✅ | ✅ | ✅ | `/t0x0102_action` |
| `ArmAction` | ✅ | ✅ | ✅ | `/t0x0103_` |
| `SpearAction` | ✅ | ✅ | ✅ | `/t0x0104_` |

## 构建

依赖：

- Ubuntu 22.04
- ROS 2 Humble
- BehaviorTree.CPP v4
- `nlohmann_json`
- libusb-1.0
- Python 3.10+

构建：

```bash
cd robocon26_r2_core
colcon build --symlink-install
source install/setup.bash
```

只构建核心包：

```bash
colcon build --symlink-install --packages-select r2_interfaces r2_bt r2_hardware
source install/setup.bash
```

## 运行

仅启动 BT 引擎：

```bash
ros2 launch r2_bt bt_engine.launch.py
```

单独调试梅林区树时可以把参数 `tree_file` 改为 `meilin_stage.xml`。

仿真（含所有 Action Server）：

```bash
ros2 launch r2_bringup r2_sim.launch.py
```

实车：

```bash
ros2 launch r2_bringup r2_full.launch.py
```

## USB 数据通道

`ares_usb` 将 ROS 2 topic 与下位机 DataID 做透传映射：

| Topic / DataID | 方向 | 说明 |
|----------------|------|------|
| `/t0x0101` | ROS → 下位机 | 底盘运动指令 `[vx, vy, vyaw]` |
| `/t0x0102_action` | ROS → 下位机 | 四轮悬挂目标高度 `[h0, h1, h2, h3]` |
| `/t0x0103_` | ROS → 下位机 | 机械臂控制指令 |
| `/t0x0104_` | ROS → 下位机 | 矛头机构控制指令 |
| `/r0x0201` | 下位机 → ROS | 底盘速度反馈 |
| `/r0x0202` | 下位机 → ROS | 光电开关 + 轮高反馈 |
| `/r0x0203` | 下位机 → ROS | 机械臂状态反馈 |
| `/r0x0204` | 下位机 → ROS | 矛头机构状态反馈 |
| `/sensor_distances` | 传感器 → ROS | 8 路 TOF 距离 |
| `/odom_world` | 定位 → ROS | 世界坐标位姿 |

## 开发原则

1. `r2_bt` 只调度语义 Action，不直接控制电机、串口、GPIO 或 PWM。
2. 外部规划模块负责路线合法性，BT 按已规划 segment 顺序执行。
3. 行为树固定加载，运行时只更新 `segment_queue` 和黑板字段。
4. JSON 使用结构化解析库，不手写字符串解析。
5. Condition 节点必须瞬间返回；耗时工作使用 `StatefulActionNode`。
6. Action Client 使用异步发送、异步结果回调和异步取消。
7. 失败日志和 Groot2 黑板必须包含当前 segment、Action 和错误原因。
