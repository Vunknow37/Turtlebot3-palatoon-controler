#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math


class StraightLineAvoid(Node):

    def __init__(self):
        super().__init__('straight_line_avoid')

        # ---------- RELATIVE TOPICS (NAMESPACE SAFE) ----------
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # ---------- STATE ----------
        self.front_dist = float('inf')
        self.current_yaw = None
        self.initial_yaw = None

        self.state = "FORWARD"
        self.turn_start_time = None
        self.bypass_start_time = None

        self.get_logger().info("✅ StraightLineAvoid node STARTED")

    # ---------- LASER CALLBACK ----------
    def scan_cb(self, msg):
        front_ranges = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Front ±15 degrees
            if abs(angle) < math.radians(15):
                if msg.range_min < r < msg.range_max:
                    front_ranges.append(r)

        self.front_dist = min(front_ranges) if front_ranges else float('inf')

    # ---------- ODOM CALLBACK ----------
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_yaw = yaw

        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.get_logger().info(f"Initial yaw locked: {yaw:.3f}")

    # ---------- CONTROL LOOP ----------
    def control_loop(self):
        cmd = Twist()

        # ✅ MOVE EVEN IF ODOM NOT READY
        if self.current_yaw is None or self.initial_yaw is None:
            cmd.linear.x = 0.12
            self.cmd_pub.publish(cmd)
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]
        yaw_error = self.normalize_angle(self.initial_yaw - self.current_yaw)

        # ---------- FORWARD ----------
        if self.state == "FORWARD":
            if self.front_dist < 0.7:
                self.state = "TURN"
                self.turn_start_time = now
                self.get_logger().info("🚧 Obstacle detected → TURN")
            else:
                cmd.linear.x = 0.15
                cmd.angular.z = 1.0 * yaw_error

        # ---------- TURN ----------
        elif self.state == "TURN":
            cmd.angular.z = 0.6
            if now - self.turn_start_time > 2.0:
                self.state = "BYPASS"
                self.bypass_start_time = now
                self.get_logger().info("↪️ TURN done → BYPASS")

        # ---------- BYPASS ----------
        elif self.state == "BYPASS":
            cmd.linear.x = 0.15
            if now - self.bypass_start_time > 2.5:
                self.state = "REALIGN"
                self.get_logger().info("➡️ BYPASS done → REALIGN")

        # ---------- REALIGN ----------
        elif self.state == "REALIGN":
            if abs(yaw_error) > 0.05:
                cmd.angular.z = 0.8 * yaw_error
            else:
                self.state = "FORWARD"
                self.get_logger().info("✅ REALIGNED → FORWARD")

        self.cmd_pub.publish(cmd)

    # ---------- UTILS ----------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = StraightLineAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
