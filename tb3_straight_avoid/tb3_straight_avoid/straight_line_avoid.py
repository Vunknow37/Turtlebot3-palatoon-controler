#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
from tf_transformations import euler_from_quaternion

using_namespace = "TB3_1"


class StraightLineAvoid(Node):

    def __init__(self):
        super().__init__('straight_line_avoid')

        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{using_namespace}/cmd_vel',
            10
        )

        self.create_subscription(
            LaserScan,
            f'/{using_namespace}/scan',
            self.scan_cb,
            10
        )

        self.create_subscription(
            Odometry,
            f'/{using_namespace}/odom',
            self.odom_cb,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.front_dist = float('inf')
        self.current_yaw = None
        self.initial_yaw = None

        self.state = "FORWARD"
        self.turn_start_time = None
        self.bypass_start_time = None

        self.get_logger().info("Straight line avoid node started")

    # ---------------- CALLBACKS ----------------

    def scan_cb(self, msg):
        front_indices = list(range(0, 10)) + list(range(len(msg.ranges)-10, len(msg.ranges)))
        valid = [msg.ranges[i] for i in front_indices if msg.ranges[i] > 0.0]

        self.front_dist = min(valid) if valid else float('inf')

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.get_logger().info("Initial yaw saved")

    # ---------------- CONTROL ----------------

    def control_loop(self):
        if self.current_yaw is None or self.initial_yaw is None:
            return

        cmd = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]
        yaw_error = self.normalize_angle(self.initial_yaw - self.current_yaw)

        # -------- FORWARD --------
        if self.state == "FORWARD":
            if self.front_dist < 0.5:
                self.state = "TURN"
                self.turn_start_time = now
                self.get_logger().info("Obstacle detected → TURN")
            else:
                cmd.linear.x = 0.2
                cmd.angular.z = 0.8 * yaw_error

        # -------- TURN --------
        elif self.state == "TURN":
            cmd.angular.z = 0.5
            if now - self.turn_start_time > 2.0:
                self.state = "BYPASS"
                self.bypass_start_time = now
                self.get_logger().info("Turn complete → BYPASS")

        # -------- BYPASS --------
        elif self.state == "BYPASS":
            cmd.linear.x = 0.2
            if now - self.bypass_start_time > 2.5:
                self.state = "REALIGN"
                self.get_logger().info("Bypass complete → REALIGN")

        # -------- REALIGN --------
        elif self.state == "REALIGN":
            if abs(yaw_error) > 0.05:
                cmd.angular.z = 0.6 * yaw_error
            else:
                self.state = "FORWARD"
                self.get_logger().info("Yaw aligned → FORWARD")

        self.cmd_pub.publish(cmd)

    # ---------------- UTILS ----------------

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
