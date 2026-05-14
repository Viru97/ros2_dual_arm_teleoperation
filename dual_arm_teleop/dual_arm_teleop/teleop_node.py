import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64, Int8
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

        self.declare_parameter("arm", "both")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("show_debug_image", True)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.6)
        self.declare_parameter("deadzone", 0.06)
        self.declare_parameter("filter_alpha", 0.3)
        # self.declare_parameter("motion_full_scale", 0.25)
        self.declare_parameter("no_hand_pause_timeout", 0.4)

        requested_arm = self.get_parameter("arm").value
        if requested_arm not in ("left", "right", "both"):
            raise ValueError("arm must be 'left', 'right', or 'both'")
        self.active_arms = ("left", "right") if requested_arm == "both" else (requested_arm,)
        self.show_debug_image = bool(self.get_parameter("show_debug_image").value)
        filter_alpha = float(self.get_parameter("filter_alpha").value)

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
            "left": {
                "start": self.create_client(Trigger, '/left_servo_node/start_servo'),
                "pause": self.create_client(Trigger, '/left_servo_node/pause_servo'),
                "unpause": self.create_client(Trigger, '/left_servo_node/unpause_servo'),
            },
            "right": {
                "start": self.create_client(Trigger, '/right_servo_node/start_servo'),
                "pause": self.create_client(Trigger, '/right_servo_node/pause_servo'),
                "unpause": self.create_client(Trigger, '/right_servo_node/unpause_servo'),
            },
        }
        self.status_subs = {
            "left": self.create_subscription(Int8, '/left_servo_node/status', self.left_status_cb, 10),
            "right": self.create_subscription(Int8, '/right_servo_node/status', self.right_status_cb, 10),
        }
        self.servo_status = {"left": 0, "right": 0}
        self.started_servos = set()
        self.paused_servos = set()
        self.requested_servo_services = set()
        self.last_hand_seen_time = {"left": None, "right": None}
        self.last_status_log_time = {"left": 0.0, "right": 0.0}
        # self.neutral_hand_pose = {"left": None, "right": None}
        # self.singularity_latched = set()

        self.tracker = HandTracker(max_num_hands=2)
        self.filters = {
            "left": {
                "x": EMAFilter(alpha=filter_alpha),
                "y": EMAFilter(alpha=filter_alpha),
                "roll": EMAFilter(alpha=filter_alpha),
            },
            "right": {
                "x": EMAFilter(alpha=filter_alpha),
                "y": EMAFilter(alpha=filter_alpha),
                "roll": EMAFilter(alpha=filter_alpha),
            },
        }

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        # self.motion_full_scale = float(self.get_parameter("motion_full_scale").value)
        self.no_hand_pause_timeout = float(self.get_parameter("no_hand_pause_timeout").value)
        self.last_log_time = 0.0

        camera_index = int(self.get_parameter("camera_index").value)
        self.cap = cv2.VideoCapture(camera_index)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info(
            f"Hand teleop started for {self.active_arms}. Center hand to stop; move hand to servo."
        )

    def left_status_cb(self, msg):
        self.servo_status["left"] = msg.data
        self.log_servo_status("left", msg.data)

    def right_status_cb(self, msg):
        self.servo_status["right"] = msg.data
        self.log_servo_status("right", msg.data)

    def log_servo_status(self, arm, status):
        now = time.monotonic()
        if status == 2 and now - self.last_status_log_time[arm] > 2.0:
            self.last_status_log_time[arm] = now
            # self.singularity_latched.add(arm)
            # self.publish_twist(arm, 0.0, 0.0, 0.0)
            # self.request_servo_service(arm, "pause")
            self.get_logger().warn(
                f"{arm.capitalize()} Servo hit a singularity stop. "
                "Hand teleop is latched off for that arm. Remove your hand to reset the latch, "
                "or plan back to a bent ready pose."
            )

    def request_servo_service(self, arm, service_name):
        key = (arm, service_name)
        client = self.servo_clients[arm][service_name]
        if key in self.requested_servo_services or not client.service_is_ready():
            return False

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done, arm_name=arm, service=service_name: self.handle_servo_service_response(
                arm_name, service, done
            )
        )
        self.requested_servo_services.add(key)
        return True

    def handle_servo_service_response(self, arm, service_name, future):
        self.requested_servo_services.discard((arm, service_name))
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Could not call {service_name} on {arm} Servo: {exc}")
            return

        if not response.success:
            self.get_logger().warn(
                f"{arm.capitalize()} Servo {service_name} request failed: {response.message}"
            )
            return

        if service_name == "start":
            self.started_servos.add(arm)
            self.paused_servos.discard(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo started for hand teleop.")
        elif service_name == "pause":
            self.paused_servos.add(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo paused; MoveIt can use the controller.")
        elif service_name == "unpause":
            self.paused_servos.discard(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo unpaused for hand teleop.")

    def ensure_servo_active(self, arm):
        if arm not in self.started_servos:
            self.request_servo_service(arm, "start")
            return
        if arm in self.paused_servos:
            self.request_servo_service(arm, "unpause")

    def pause_servo_if_idle(self, arm, now):
        last_seen = self.last_hand_seen_time[arm]
        if arm not in self.started_servos or arm in self.paused_servos:
            return
        if last_seen is None or now - last_seen > self.no_hand_pause_timeout:
            self.publish_twist(arm, 0.0, 0.0, 0.0)
            self.request_servo_service(arm, "pause")

    def timer_callback(self):
        now = time.monotonic()
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn("Camera frame was empty; publishing halt command.")
            self.publish_halt_commands()
            for arm in self.active_arms:
                self.pause_servo_if_idle(arm, now)
            return

        frame = cv2.flip(frame, 1)
        annotated_frame, results = self.tracker.process_frame(frame)

        commands = {
            "left": {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "gripper": None, "seen": False},
            "right": {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "gripper": None, "seen": False},
        }
        if results.multi_hand_landmarks:
            assignments = self.assign_hands(results.multi_hand_landmarks)
            for arm, hand in assignments.items():
                if arm not in self.active_arms:
                    continue
                # if arm in self.singularity_latched:
                #     commands[arm]["seen"] = True
                #     self.last_hand_seen_time[arm] = now
                #     continue
                commands[arm] = self.hand_to_command(arm, hand)
                commands[arm]["seen"] = True
                self.last_hand_seen_time[arm] = now
                self.ensure_servo_active(arm)

        for arm in self.active_arms:
            command = commands[arm]
            if not command["seen"]:
                # self.handle_hand_lost(arm)
                self.publish_twist(arm, 0.0, 0.0, 0.0)
                self.pause_servo_if_idle(arm, now)
                continue
            # if arm in self.singularity_latched:
            #     self.publish_twist(arm, 0.0, 0.0, 0.0)
            #     continue
            self.publish_twist(arm, command["vel_y"], command["vel_z"], command["yaw"])
            if command["gripper"] is not None:
                self.publish_gripper(arm, command["gripper"])
            if command["seen"]:
                self.publish_target_pose(arm, command["vel_y"], command["vel_z"], command["yaw"])

        self.log_commands(commands)

        if self.show_debug_image:
            self.draw_debug_overlay(annotated_frame, commands)
            cv2.imshow('Hand Teleop', annotated_frame)
            cv2.waitKey(1)

    def assign_hands(self, hand_landmarks):
        if len(self.active_arms) == 1:
            return {self.active_arms[0]: hand_landmarks[0]}

        hands = sorted(hand_landmarks, key=lambda hand: hand.landmark[0].x)
        if len(hands) == 1:
            wrist_x = hands[0].landmark[0].x
            return {"left" if wrist_x < 0.5 else "right": hands[0]}
        return {"left": hands[0], "right": hands[-1]}

    def hand_to_command(self, arm, hand):
        raw_x = hand.landmark[8].x
        raw_y = hand.landmark[8].y

        thumb_tip = hand.landmark[4]
        index_tip = hand.landmark[8]
        pinch = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)

        smooth_x = self.filters[arm]["x"].update(raw_x)
        smooth_y = self.filters[arm]["y"].update(raw_y)

        y_axis = self.normalized_axis(0.5 - smooth_x)
        z_axis = self.normalized_axis(0.5 - smooth_y)
        vel_y = y_axis * self.max_linear_speed
        vel_z = z_axis * self.max_linear_speed

        index_mcp = hand.landmark[5]
        pinky_mcp = hand.landmark[17]
        raw_roll = pinky_mcp.y - index_mcp.y
        smooth_roll = self.filters[arm]["roll"].update(raw_roll)
        yaw_axis = self.normalized_axis(smooth_roll, deadzone=0.03, full_scale=0.15)
        #
        # if self.neutral_hand_pose[arm] is None:
        #     self.neutral_hand_pose[arm] = {
        #         "x": smooth_x,
        #         "y": smooth_y,
        #         "roll": smooth_roll,
        #     }
        #     self.get_logger().info(f"{arm.capitalize()} hand neutral point captured.")
        #     pinch = self.pinch_distance(hand)
        #     gripper = self.clamp((pinch - 0.03) / 0.15, 0.0, 1.0)
        #     return {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "gripper": gripper}
        #
        # neutral = self.neutral_hand_pose[arm]
        # y_axis = self.normalized_axis(neutral["x"] - smooth_x, full_scale=self.motion_full_scale)
        # z_axis = self.normalized_axis(neutral["y"] - smooth_y, full_scale=self.motion_full_scale)
        # vel_y = y_axis * self.max_linear_speed
        # vel_z = z_axis * self.max_linear_speed
        #
        # yaw_axis = self.normalized_axis(smooth_roll - neutral["roll"], deadzone=0.03, full_scale=0.15)
        yaw = yaw_axis * self.max_angular_speed

        # pinch = self.pinch_distance(hand)
        gripper = self.clamp((pinch - 0.03) / 0.15, 0.0, 1.0)

        return {"vel_y": vel_y, "vel_z": vel_z, "yaw": yaw, "gripper": gripper}

    def apply_deadzone(self, value, deadzone=None):
        active_deadzone = self.deadzone if deadzone is None else deadzone
        return 0.0 if abs(value) < active_deadzone else value

    def normalized_axis(self, value, deadzone=None, full_scale=0.5):
        value = self.apply_deadzone(value, deadzone)
        if value == 0.0:
            return 0.0
        return self.clamp(value / full_scale, -1.0, 1.0)

    # def pinch_distance(self, hand):
    #     thumb_tip = hand.landmark[4]
    #     index_tip = hand.landmark[8]
    #     return math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)

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

    def publish_halt_commands(self):
        for arm in self.active_arms:
            self.publish_twist(arm, 0.0, 0.0, 0.0)

    # def handle_hand_lost(self, arm):
    #     if self.neutral_hand_pose[arm] is not None:
    #         self.neutral_hand_pose[arm] = None
    #         self.get_logger().info(f"{arm.capitalize()} hand neutral point reset.")
    #     if arm in self.singularity_latched:
    #         self.singularity_latched.discard(arm)
    #         self.get_logger().info(f"{arm.capitalize()} singularity latch reset after hand removal.")

    def draw_debug_overlay(self, frame, commands):
        height, width = frame.shape[:2]
        cv2.line(frame, (width // 2, 0), (width // 2, height), (80, 80, 80), 1)
        cv2.circle(frame, (width // 2, height // 2), 32, (0, 255, 255), 1)
        y = 28
        for arm in self.active_arms:
            command = commands[arm]
            color = (0, 220, 0) if command["seen"] else (120, 120, 120)
            text = (
                f"{arm}: y={command['vel_y']:.2f} "
                f"z={command['vel_z']:.2f} yaw={command['yaw']:.2f}"
            )
            cv2.putText(frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            y += 24

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_halt_commands()
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
