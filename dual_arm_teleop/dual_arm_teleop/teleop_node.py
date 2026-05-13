import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import cv2
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

from dual_arm_teleop.hand_tracker import HandTracker
from dual_arm_teleop.signal_filter import EMAFilter

class TeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.twist_pubs = {
            "left": self.create_publisher(TwistStamped, '/left_servo_node/delta_twist_cmds', qos_profile),
            "right": self.create_publisher(TwistStamped, '/right_servo_node/delta_twist_cmds', qos_profile),
        }
        self.legacy_left_twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', qos_profile
        )
        self.pose_pubs = {
            "left": self.create_publisher(PoseStamped, '/teleop/left_ee_target', 10),
            "right": self.create_publisher(PoseStamped, '/teleop/right_ee_target', 10),
        }
        self.gripper_pubs = {
            "left": self.create_publisher(Float64, '/teleop/left_gripper_target', 10),
            "right": self.create_publisher(Float64, '/teleop/right_gripper_target', 10),
        }
        self.command_frames = {
            "left": "left_base_link",
            "right": "right_base_link",
        }

        self.servo_clients = {
            "left": self.create_client(Trigger, '/left_servo_node/start_servo'),
            "right": self.create_client(Trigger, '/right_servo_node/start_servo'),
        }
        self.started_servos = set()
        self.requested_servos = set()
        self.start_servo_timer = self.create_timer(1.0, self.try_start_servos)

        self.tracker = HandTracker(max_num_hands=2)
        self.filters = {
            "left": {"x": EMAFilter(alpha=0.2), "y": EMAFilter(alpha=0.2), "roll": EMAFilter(alpha=0.2)},
            "right": {"x": EMAFilter(alpha=0.2), "y": EMAFilter(alpha=0.2), "roll": EMAFilter(alpha=0.2)},
        }

        self.max_linear_speed = 0.25
        self.max_angular_speed = 0.6
        self.deadzone = 0.06
        self.last_log_time = 0.0

        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info(
            "Dual hand teleop started. Left camera half drives left arm, right half drives right arm."
        )

    def try_start_servos(self):
        for arm, client in self.servo_clients.items():
            if arm in self.started_servos or arm in self.requested_servos or not client.service_is_ready():
                continue
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda done, arm_name=arm: self.handle_start_response(arm_name, done))
            self.requested_servos.add(arm)

    def handle_start_response(self, arm, future):
        self.requested_servos.discard(arm)
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Could not start {arm} Servo: {exc}")
            return
        if response.success:
            self.started_servos.add(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo started.")
        else:
            self.get_logger().warn(f"{arm.capitalize()} Servo start request failed: {response.message}")
        if len(self.started_servos) == len(self.servo_clients):
            self.destroy_timer(self.start_servo_timer)

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success: return

        frame = cv2.flip(frame, 1)
        annotated_frame, results = self.tracker.process_frame(frame)

        commands = {
            "left": {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "gripper": None, "seen": False},
            "right": {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "gripper": None, "seen": False},
        }
        if results.multi_hand_landmarks:
            assignments = self.assign_hands(results.multi_hand_landmarks)
            for arm, hand in assignments.items():
                commands[arm] = self.hand_to_command(arm, hand)
                commands[arm]["seen"] = True

        for arm, command in commands.items():
            self.publish_twist(arm, command["vel_y"], command["vel_z"], command["yaw"])
            if command["gripper"] is not None:
                self.publish_gripper(arm, command["gripper"])
            if command["seen"]:
                self.publish_target_pose(arm, command["vel_y"], command["vel_z"], command["yaw"])

        self.log_commands(commands)

        cv2.imshow('Teleop Joystick', annotated_frame)
        cv2.waitKey(1)

    def assign_hands(self, hand_landmarks):
        hands = sorted(hand_landmarks, key=lambda hand: hand.landmark[0].x)
        if len(hands) == 1:
            wrist_x = hands[0].landmark[0].x
            return {"left" if wrist_x < 0.5 else "right": hands[0]}
        return {"left": hands[0], "right": hands[-1]}

    def hand_to_command(self, arm, hand):
        raw_x = hand.landmark[8].x
        raw_y = hand.landmark[8].y

        smooth_x = self.filters[arm]["x"].update(raw_x)
        smooth_y = self.filters[arm]["y"].update(raw_y)

        dy = self.apply_deadzone(0.5 - smooth_x)
        dz = self.apply_deadzone(0.5 - smooth_y)
        vel_y = self.clamp(dy * self.max_linear_speed, -self.max_linear_speed, self.max_linear_speed)
        vel_z = self.clamp(dz * self.max_linear_speed, -self.max_linear_speed, self.max_linear_speed)

        index_mcp = hand.landmark[5]
        pinky_mcp = hand.landmark[17]
        raw_roll = pinky_mcp.y - index_mcp.y
        smooth_roll = self.filters[arm]["roll"].update(raw_roll)
        yaw = self.clamp(self.apply_deadzone(smooth_roll, 0.03) * 4.0, -self.max_angular_speed, self.max_angular_speed)

        thumb_tip = hand.landmark[4]
        index_tip = hand.landmark[8]
        pinch = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)
        gripper = self.clamp((pinch - 0.03) / 0.15, 0.0, 1.0)

        return {"vel_y": vel_y, "vel_z": vel_z, "yaw": yaw, "gripper": gripper}

    def apply_deadzone(self, value, deadzone=None):
        active_deadzone = self.deadzone if deadzone is None else deadzone
        return 0.0 if abs(value) < active_deadzone else value

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def publish_twist(self, arm, vel_y, vel_z, yaw):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frames[arm]

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = vel_y
        msg.twist.linear.z = vel_z
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = yaw

        self.twist_pubs[arm].publish(msg)
        if arm == "left":
            self.legacy_left_twist_pub.publish(msg)

    def publish_gripper(self, arm, openness):
        msg = Float64()
        msg.data = float(openness)
        self.gripper_pubs[arm].publish(msg)

    def publish_target_pose(self, arm, vel_y, vel_z, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = 0.45
        msg.pose.position.y = (0.4 if arm == "left" else -0.4) + vel_y
        msg.pose.position.z = 0.95 + vel_z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.pose_pubs[arm].publish(msg)

    def log_commands(self, commands):
        now = time.monotonic()
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        left = commands["left"]
        right = commands["right"]
        self.get_logger().info(
            "Teleop speed "
            f"L(y={left['vel_y']:.2f}, z={left['vel_z']:.2f}, yaw={left['yaw']:.2f}) "
            f"R(y={right['vel_y']:.2f}, z={right['vel_z']:.2f}, yaw={right['yaw']:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
