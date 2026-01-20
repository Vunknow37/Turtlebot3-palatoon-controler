#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class CircleController(Node):
    def __init__(self):
        super().__init__("circle_controller")

        # ---------------- Params ----------------
        self.declare_parameter("namespace", "")
        self.declare_parameter("radius", 1.0)        # desired circle radius [m]
        self.declare_parameter("linear_speed", 0.1)  # forward speed [m/s]
        self.declare_parameter("direction", "ccw")   # "ccw" or "cw"

        ns = self.get_parameter("namespace").value
        self.radius = float(self.get_parameter("radius").value)
        self.v = float(self.get_parameter("linear_speed").value)
        direction_str = str(self.get_parameter("direction").value).lower()
        self.turn_ccw = (direction_str != "cw")

        # ---------------- Topics ----------------
        if ns == "":
            cmd_topic = "/cmd_vel"
            odom_topic = "/odom"
        else:
            cmd_topic = f"/{ns}/cmd_vel"
            odom_topic = f"/{ns}/odom"

        self.get_logger().info(f"Publishing circle motion to: {cmd_topic}")
        self.get_logger().info(f"Using odometry from: {odom_topic}")
        self.get_logger().info(
            f"Desired radius: {self.radius} m, speed: {self.v} m/s, "
            f"direction: {'CCW' if self.turn_ccw else 'CW'}"
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        # pose & circle center
        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.center_x = None
        self.center_y = None

        # heading feedback gain
        self.k_heading = 3.0

        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.timer_callback)

        # for occasional debug prints
        self._print_count = 0

    # ------------- ODOM CALLBACK -------------
    def odom_callback(self, msg: Odometry):
        # extract pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.yaw = yaw
        self.have_pose = True

        # On first odom, define circle center using desired radius
        if self.center_x is None or self.center_y is None:
            # Center is to the LEFT of robot for CCW, RIGHT for CW
            sign = 1.0 if self.turn_ccw else -1.0

            # left of heading = (+90°) => (-R*sin(yaw), +R*cos(yaw))
            self.center_x = self.x - sign * self.radius * math.sin(self.yaw)
            self.center_y = self.y + sign * self.radius * math.cos(self.yaw)

            self.get_logger().info(
                f"Circle center set at ({self.center_x:.3f}, {self.center_y:.3f}) "
                f"from initial pose ({self.x:.3f}, {self.y:.3f}, yaw={self.yaw:.3f})"
            )

    # ------------- MAIN CONTROL LOOP -------------
    def timer_callback(self):
        if not self.have_pose or self.center_x is None:
            # wait for odom + center
            return

        # current radius from center
        dx = self.x - self.center_x
        dy = self.y - self.center_y
        r = math.sqrt(dx*dx + dy*dy) + 1e-6  # avoid divide by zero

        # angle from center -> robot
        alpha = math.atan2(dy, dx)

        # desired robot heading is tangent to the circle
        if self.turn_ccw:
            desired_yaw = alpha + math.pi / 2.0   # CCW tangent
            base_w = self.v / self.radius        # base angular speed
        else:
            desired_yaw = alpha - math.pi / 2.0   # CW tangent
            base_w = -self.v / self.radius

        # heading error
        heading_error = self.normalize_angle(desired_yaw - self.yaw)

        # controller: base circle + heading correction
        w = base_w + self.k_heading * heading_error

        # optional: clamp angular velocity to something safe
        max_w = 1.5  # rad/s, adjust if needed
        if w > max_w:
            w = max_w
        elif w < -max_w:
            w = -max_w

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

        # occasional debug
        self._print_count += 1
        if self._print_count % 20 == 0:  # every ~2 seconds at 10 Hz
            self.get_logger().info(
                f"current r = {r:.2f} m, target R = {self.radius:.2f} m, "
                f"heading_error = {heading_error:.2f} rad"
            )

    # ------------- UTILITIES -------------
    @staticmethod
    def normalize_angle(angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        """Publish zero cmd_vel to stop the robot."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(stop_msg)
        self.get_logger().info("Sent stop command (0 velocity).")


def main(args=None):
    rclpy.init(args=args)
    node = CircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
