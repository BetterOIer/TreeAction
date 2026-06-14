#!/usr/bin/env python3
"""
MoveToPose Action Server

接收单个目标位姿 (x, y, yaw)，使用 PID 控制驱动底盘到达目标。
发布 cmd_vel 到 /t0x0101（Float32MultiArray: [vx, vy, vyaw]）。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from r2_interfaces.action import MoveToPose
from std_msgs.msg import Float32MultiArray

from r2_hardware.pid_controller import PIDController
from r2_hardware.transform_utils import yaw_from_quaternion


class MoveToPoseActionServer(Node):
    def __init__(self):
        super().__init__("move_to_pose_action_server")

        self.declare_parameter("control_frequency", 50.0)
        self.declare_parameter("arrive_threshold", 0.12)
        self.declare_parameter("angle_threshold", 0.08)
        self.declare_parameter("odom_topic", "/odom_world")
        self.declare_parameter("cmd_vel_topic", "/t0x0101")

        self.declare_parameter("kp_angle", 4.0)
        self.declare_parameter("ki_angle", 0.0)
        self.declare_parameter("kd_angle", 0.2)
        self.declare_parameter("kp_dist", 2.5)
        self.declare_parameter("ki_dist", 0.0)
        self.declare_parameter("kd_dist", 0.1)
        self.declare_parameter("max_vel", 0.5)
        self.declare_parameter("max_angular", 2.0)

        self.control_freq = self._p("control_frequency")
        self.arrive_threshold = self._p("arrive_threshold")
        self.angle_threshold = self._p("angle_threshold")

        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.have_odom = False

        self.pid_dist = PIDController(
            self._p("kp_dist"), self._p("ki_dist"), self._p("kd_dist"),
            self._p("max_vel"),
        )
        self.pid_angle = PIDController(
            self._p("kp_angle"), self._p("ki_angle"), self._p("kd_angle"),
            self._p("max_angular"),
        )

        self.odom_sub = self.create_subscription(
            PoseStamped, self._p("odom_topic"), self._odom_callback, 10,
        )
        self.cmd_pub = self.create_publisher(
            Float32MultiArray, self._p("cmd_vel_topic"), 10,
        )

        self._action_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("MoveToPose Action Server started")

    def _p(self, name, default=None):
        val = self.get_parameter(name).value
        return val if val is not None else default

    def _odom_callback(self, msg: PoseStamped):
        self.current_pos[0] = msg.pose.position.x
        self.current_pos[1] = msg.pose.position.y
        self.current_yaw = self._normalize_angle(
            yaw_from_quaternion([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ])
        )
        self.have_odom = True

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _goal_callback(self, goal_request):
        self.get_logger().info("Accepting move_to_pose goal")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("MoveToPose goal cancelled, stopping")
        self._stop()
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle: ServerGoalHandle):
        target = goal_handle.request.target_pose
        max_speed = goal_handle.request.max_speed

        tx = target.pose.position.x
        ty = target.pose.position.y

        qx = target.pose.orientation.x
        qy = target.pose.orientation.y
        qz = target.pose.orientation.z
        qw = target.pose.orientation.w
        target_yaw = self._normalize_angle(
            yaw_from_quaternion([qx, qy, qz, qw])
        )

        self.get_logger().info(
            f"MoveToPose target: x={tx:.3f} y={ty:.3f} yaw={math.degrees(target_yaw):.1f}deg"
        )

        if not self.have_odom:
            self.get_logger().warn("No odometry yet, waiting...")

        feedback_msg = MoveToPose.Feedback()
        self.pid_dist.reset()
        self.pid_angle.reset()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop()
                return MoveToPose.Result(success=False, message="Cancelled")

            if not self.have_odom:
                await self._sleep(1.0 / self.control_freq)
                continue

            dx = tx - self.current_pos[0]
            dy = ty - self.current_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)

            desired_yaw = math.atan2(dy, dx)
            yaw_error = self._normalize_angle(desired_yaw - self.current_yaw)

            feedback_msg.distance_remaining = float(dist)
            goal_handle.publish_feedback(feedback_msg)

            if dist < self.arrive_threshold:
                yaw_error_final = self._normalize_angle(target_yaw - self.current_yaw)
                if abs(yaw_error_final) < self.angle_threshold:
                    self._stop()
                    goal_handle.succeed()
                    self.get_logger().info("MoveToPose succeeded")
                    return MoveToPose.Result(success=True, message="Arrived")

                angular_cmd = self.pid_angle.update(yaw_error_final, 0.0)
                self._publish_cmd(0.0, 0.0, angular_cmd)
                await self._sleep(1.0 / self.control_freq)
                continue

            vx = self.pid_dist.update(dist, 0.0) * max_speed
            angular = self.pid_angle.update(yaw_error, 0.0)

            vx = max(-max_speed, min(max_speed, vx))

            self._publish_cmd(vx, 0.0, angular)
            await self._sleep(1.0 / self.control_freq)

        goal_handle.abort()
        return MoveToPose.Result(success=False, message="Node shutdown")

    def _publish_cmd(self, vx, vy, vyaw):
        self.cmd_pub.publish(Float32MultiArray(data=[float(vx), float(vy), float(vyaw)]))

    def _stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)

    async def _sleep(self, sec):
        import asyncio
        await asyncio.sleep(sec)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
