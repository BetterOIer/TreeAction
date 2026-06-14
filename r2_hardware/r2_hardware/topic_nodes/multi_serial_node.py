#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多路串口距离传感器节点
从 8 个 TOF 距离传感器通过 CH9344 USB-to-UART 读取距离数据，
以 100Hz 发布到 /sensor_distances。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import math
import time


class MultiSerialPublisher(Node):
    def __init__(self):
        super().__init__('multi_serial_publisher')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_distances', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.port_names = [f'/dev/ttyCH9344USB{i}' for i in range(8)]
        self.baudrate = 230400

        self.serials = []
        self.buffers = [bytearray() for _ in range(8)]
        self.last_distances = [math.nan] * 8
        self.miss_counts = [0] * 8
        self.last_reconnect_attempt = [0.0] * 8
        self.reconnect_interval = 1.0

        for i in range(len(self.port_names)):
            self.serials.append(None)
            self._try_open_serial(i, force=True)

    def _try_open_serial(self, index, force=False):
        now = time.monotonic()
        if not force and now - self.last_reconnect_attempt[index] < self.reconnect_interval:
            return

        self.last_reconnect_attempt[index] = now
        port = self.port_names[index]
        try:
            self.serials[index] = serial.Serial(port, self.baudrate, timeout=0)
            self.buffers[index].clear()
            self.miss_counts[index] = 0
            self.get_logger().info(f'成功打开串口: {port}')
        except serial.SerialException:
            self.serials[index] = None

    def parse_packet(self, data):
        for i in range(10, 195, 15):
            if i + 15 <= len(data):
                confidence = data[i + 8]
                if confidence == 100:
                    distance = data[i] | (data[i + 1] << 8)
                    return float(distance)
        return math.nan

    def timer_callback(self):
        msg = Float32MultiArray()

        for i in range(8):
            ser = self.serials[i]

            if ser is not None and ser.is_open:
                try:
                    if ser.in_waiting > 0:
                        self.buffers[i].extend(ser.read(ser.in_waiting))

                    while len(self.buffers[i]) >= 195:
                        if self.buffers[i][0] == 0xAA:
                            packet = self.buffers[i][:195]
                            distance = self.parse_packet(packet)
                            if math.isnan(distance):
                                self.miss_counts[i] += 1
                            else:
                                self.miss_counts[i] = 0
                                self.last_distances[i] = distance
                            self.buffers[i] = self.buffers[i][195:]
                        else:
                            self.buffers[i].pop(0)

                        if self.miss_counts[i] >= 5:
                            if not math.isnan(self.last_distances[i]):
                                self.get_logger().warn(
                                    f'串口 {self.port_names[i]} 掉线，连续 5 次未读取到有效数据。')
                            self.last_distances[i] = math.nan

                    if ser is None or not ser.is_open:
                        self.last_distances[i] = math.nan
                except Exception as e:
                    self.get_logger().error(f'读取串口 {self.port_names[i]} 失败: {e}')
                    self.serials[i].close()
                    self.serials[i] = None
                    self.buffers[i].clear()
                    self.miss_counts[i] = 0
                    self.last_distances[i] = math.nan
            else:
                self.last_distances[i] = math.nan
                self._try_open_serial(i)

        msg.data = self.last_distances
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiSerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户手动终止。')
    finally:
        for ser in node.serials:
            if ser is not None and ser.is_open:
                ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
