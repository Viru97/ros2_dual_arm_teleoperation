import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


class ServoSmokeTest(Node):
    """Publish a small deterministic Servo command without using the camera."""

    def __init__(self):
        super().__init__("servo_smoke_test")

        self.declare_parameter("arm", "left")
        self.declare_parameter("pattern", "sine")
        self.declare_parameter("duration", 12.0)
        self.declare_parameter("linear_speed", 0.04)
        self.declare_parameter("angular_speed", 0.0)

        requested_arm = self.get_parameter("arm").value
        if requested_arm not in ("left", "right", "both"):
            raise ValueError("arm must be 'left', 'right', or 'both'")

        self.arms = ("left", "right") if requested_arm == "both" else (requested_arm,)
        self.pattern = self.get_parameter("pattern").value
        self.duration = float(self.get_parameter("duration").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.start_time = time.monotonic()
        self.done = False

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.twist_publishers = {
            "left": self.create_publisher(TwistStamped, "/left_servo_node/delta_twist_cmds", qos_profile),
            "right": self.create_publisher(TwistStamped, "/right_servo_node/delta_twist_cmds", qos_profile),
        }
        self.start_clients = {
            "left": self.create_client(Trigger, "/left_servo_node/start_servo"),
            "right": self.create_client(Trigger, "/right_servo_node/start_servo"),
        }
        self.command_frames = {
            "left": "left_base_link",
            "right": "right_base_link",
        }
        self.started = set()
        self.start_requests = set()

        self.start_timer = self.create_timer(0.5, self.try_start_servo)
        self.command_timer = self.create_timer(1.0 / 30.0, self.publish_commands)
        self.get_logger().info(
            f"Servo smoke test publishing {self.pattern} command for {self.arms} "
            f"for {self.duration:.1f}s."
        )

    def try_start_servo(self):
        for arm in self.arms:
            client = self.start_clients[arm]
            if arm in self.started or arm in self.start_requests or not client.service_is_ready():
                continue

            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda done, arm_name=arm: self.handle_start_response(arm_name, done))
            self.start_requests.add(arm)

    def handle_start_response(self, arm, future):
        self.start_requests.discard(arm)
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Could not start {arm} Servo: {exc}")
            return

        if response.success:
            self.started.add(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo accepted start request.")
        else:
            self.get_logger().warn(f"{arm.capitalize()} Servo start failed: {response.message}")

        if len(self.started) == len(self.arms):
            self.destroy_timer(self.start_timer)

    def publish_commands(self):
        elapsed = time.monotonic() - self.start_time
        if elapsed > self.duration:
            self.publish_halt()
            self.get_logger().info("Smoke test finished; published halt command.")
            self.done = True
            return

        for arm in self.arms:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.command_frames[arm]

            sign = 1.0 if arm == "left" else -1.0
            command = self.pattern_value(elapsed)
            msg.twist.linear.y = sign * self.linear_speed * command
            msg.twist.linear.z = 0.0
            msg.twist.angular.z = sign * self.angular_speed * command

            self.twist_publishers[arm].publish(msg)

    def publish_halt(self):
        for arm in self.arms:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.command_frames[arm]
            self.twist_publishers[arm].publish(msg)

    def pattern_value(self, elapsed):
        if self.pattern == "fixed":
            return 1.0
        if self.pattern == "square":
            return 1.0 if int(elapsed / 2.0) % 2 == 0 else -1.0
        if self.pattern == "sine":
            return math.sin(2.0 * math.pi * 0.2 * elapsed)
        raise ValueError("pattern must be 'fixed', 'square', or 'sine'")


def main(args=None):
    rclpy.init(args=args)
    node = ServoSmokeTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.publish_halt()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
