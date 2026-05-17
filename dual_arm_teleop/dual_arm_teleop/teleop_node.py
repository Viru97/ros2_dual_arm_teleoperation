import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int8
from std_srvs.srv import Trigger
import cv2
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

from dual_arm_teleop.hand_tracker import HandTracker
from dual_arm_teleop.signal_filter import EMAFilter


# Palm center: average of landmarks 0, 5, 9, 13, 17 (wrist + all MCP knuckles).
# Much more stable than index fingertip (landmark 8).
PALM_LANDMARKS = [0, 5, 9, 13, 17]

ARM_JOINTS = {
    "left": [
        "left_shoulder_pan_joint",
        "left_shoulder_lift_joint",
        "left_elbow_joint",
        "left_wrist_1_joint",
        "left_wrist_2_joint",
        "left_wrist_3_joint",
    ],
    "right": [
        "right_shoulder_pan_joint",
        "right_shoulder_lift_joint",
        "right_elbow_joint",
        "right_wrist_1_joint",
        "right_wrist_2_joint",
        "right_wrist_3_joint",
    ],
}

JOINT_LABELS = {
    "shoulder_pan_joint": "sh_pan",
    "shoulder_lift_joint": "sh_lift",
    "elbow_joint": "elbow",
    "wrist_1_joint": "wrist1",
    "wrist_2_joint": "wrist2",
    "wrist_3_joint": "wrist3",
}

SERVO_STATUS_MESSAGES = {
    -1: "Invalid Servo status",
    1: "Moving closer to a singularity, decelerating",
    2: "Very close to a singularity, emergency stop",
    3: "Close to a collision, decelerating",
    4: "Collision detected, emergency stop",
    5: "Close to a joint bound, halting",
    6: "Moving away from a singularity, decelerating",
}

HARD_STOP_SERVO_STATUSES = {2, 4, 5}


def palm_center(hand):
    xs = [hand.landmark[i].x for i in PALM_LANDMARKS]
    ys = [hand.landmark[i].y for i in PALM_LANDMARKS]
    return sum(xs) / len(xs), sum(ys) / len(ys)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop_node')

        self.declare_parameter("arm", "both")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("show_debug_image", True)
        # Fast enough for a visible demo while still below typical teach-mode speeds.
        self.declare_parameter("max_linear_speed", 0.45)
        self.declare_parameter("max_angular_speed", 1.2)
        # Tighter deadzone (was 0.06) so motion starts sooner
        self.declare_parameter("deadzone", 0.04)
        # EMA smoothing for palm pose and roll. Higher follows the operator sooner.
        self.declare_parameter("filter_alpha", 0.50)
        self.declare_parameter("no_hand_pause_timeout", 0.4)
        # Full-scale hand displacement from centre that maps to max speed.
        # Smaller value = less wrist travel needed for full speed.
        # 0.24 means moving hand 24% of pane width from centre = max speed.
        self.declare_parameter("motion_full_scale", 0.24)
        # Velocity ramp: max fractional change per control tick (avoids jerk).
        # 0.10 ramps from 0->100% in ~10 ticks (~100 ms at 100 Hz).
        self.declare_parameter("ramp_rate", 0.10)
        # Throttle repeated Servo warning logs while still printing the current
        # joint pose often enough to diagnose collision/singularity bottlenecks.
        self.declare_parameter("servo_status_log_period", 2.0)
        self.declare_parameter("operator_lock_enabled", True)
        self.declare_parameter("operator_acquire_radius", 0.45)
        self.declare_parameter("operator_max_jump", 0.30)
        self.declare_parameter("operator_lock_log_period", 2.0)
        self.declare_parameter("safety_latch_enabled", True)
        self.declare_parameter("split_control_window", True)
        self.declare_parameter("invert_lateral_axis", True)
        self.declare_parameter("swap_control_panes", True)

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
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.servo_status = {"left": 0, "right": 0}
        self.current_joint_positions = {}
        self.started_servos = set()
        self.paused_servos = set()
        self.requested_servo_services = set()
        self.last_hand_seen_time = {"left": None, "right": None}
        self.last_status_log_time = {"left": {}, "right": {}}
        self.last_operator_palm = {"left": None, "right": None}
        self.last_operator_lock_log_time = {"left": 0.0, "right": 0.0}
        self.safety_latches = {"left": None, "right": None}

        self.tracker = HandTracker(max_num_hands=2)

        # Per-arm EMA filters: palm x, palm y, hand roll
        self.filters = {
            arm: {
                "x": EMAFilter(alpha=filter_alpha),
                "y": EMAFilter(alpha=filter_alpha),
                "roll": EMAFilter(alpha=filter_alpha),
            }
            for arm in ("left", "right")
        }

        # Velocity ramping: track previous output velocity per arm
        self.prev_vel = {arm: {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0} for arm in ("left", "right")}
        self.last_published_commands = {
            arm: {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0}
            for arm in ("left", "right")
        }

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.motion_full_scale = float(self.get_parameter("motion_full_scale").value)
        self.ramp_rate = float(self.get_parameter("ramp_rate").value)
        self.no_hand_pause_timeout = float(self.get_parameter("no_hand_pause_timeout").value)
        self.servo_status_log_period = float(self.get_parameter("servo_status_log_period").value)
        self.operator_lock_enabled = bool(self.get_parameter("operator_lock_enabled").value)
        self.operator_acquire_radius = float(self.get_parameter("operator_acquire_radius").value)
        self.operator_max_jump = float(self.get_parameter("operator_max_jump").value)
        self.operator_lock_log_period = float(self.get_parameter("operator_lock_log_period").value)
        self.safety_latch_enabled = bool(self.get_parameter("safety_latch_enabled").value)
        self.split_control_window = bool(self.get_parameter("split_control_window").value)
        self.invert_lateral_axis = bool(self.get_parameter("invert_lateral_axis").value)
        self.swap_control_panes = bool(self.get_parameter("swap_control_panes").value)
        self.last_log_time = 0.0

        camera_index = int(self.get_parameter("camera_index").value)
        self.cap = cv2.VideoCapture(camera_index)
        # Request 60 fps from camera where supported to reduce input latency
        self.cap.set(cv2.CAP_PROP_FPS, 60)

        # Timer at ~100 Hz; matches MoveIt Servo publish_period for smoother RViz motion.
        self.timer = self.create_timer(0.010, self.timer_callback)
        self.get_logger().info(
            f"Hand teleop started for {self.active_arms}. "
            "Center hand to stop; move hand to servo. "
            f"max_linear={self.max_linear_speed} m/s, "
            f"max_angular={self.max_angular_speed} rad/s, "
            f"full_scale={self.motion_full_scale}, "
            f"ramp_rate={self.ramp_rate}, "
            f"split_control_window={self.split_control_window}, "
            f"invert_lateral_axis={self.invert_lateral_axis}, "
            f"swap_control_panes={self.swap_control_panes}"
        )

    # ------------------------------------------------------------------ #
    # Servo status callbacks
    # ------------------------------------------------------------------ #

    def left_status_cb(self, msg):
        self.handle_servo_status("left", msg.data)

    def right_status_cb(self, msg):
        self.handle_servo_status("right", msg.data)

    def handle_servo_status(self, arm, status):
        self.servo_status[arm] = status
        if self.safety_latch_enabled and status in HARD_STOP_SERVO_STATUSES:
            self.engage_safety_latch(arm, status)
        self.log_servo_status(arm, status)

    def engage_safety_latch(self, arm, status):
        if self.safety_latches[arm] is not None:
            return
        reason = SERVO_STATUS_MESSAGES.get(status, f"Servo status {status}")
        self.safety_latches[arm] = reason
        self.publish_twist(arm, 0.0, 0.0, 0.0)
        self.request_servo_service(arm, "pause")
        self.get_logger().error(
            f"{arm.capitalize()} safety latch engaged: {reason}. "
            "Hand commands are disabled for this arm. Remove your hand from view "
            "to clear the latch, then reacquire control from the centre."
        )

    def release_safety_latch_if_idle(self, arm, now):
        if self.safety_latches[arm] is None:
            return
        last_seen = self.last_hand_seen_time[arm]
        if last_seen is not None and now - last_seen <= self.no_hand_pause_timeout:
            return
        self.safety_latches[arm] = None
        self.last_operator_palm[arm] = None
        self.reset_arm_filters(arm)
        self.get_logger().info(
            f"{arm.capitalize()} safety latch cleared. Move hand near centre to resume."
        )

    def reset_arm_filters(self, arm):
        for filt in self.filters[arm].values():
            filt.previous_val = None

    def joint_state_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def log_servo_status(self, arm, status):
        if status == 0:
            return

        now = time.monotonic()
        last_log_time = self.last_status_log_time[arm].get(status, 0.0)
        if now - last_log_time < self.servo_status_log_period:
            return

        self.last_status_log_time[arm][status] = now
        status_text = SERVO_STATUS_MESSAGES.get(status, f"Unknown Servo status {status}")
        self.get_logger().warn(
            f"{arm.capitalize()} Servo status {status}: {status_text}. "
            f"{self.format_joint_angles(arm)}. "
            f"{self.format_last_command(arm)}"
        )

    def format_joint_angles(self, arm):
        parts = []
        missing = []
        prefix = f"{arm}_"
        for joint_name in ARM_JOINTS[arm]:
            position = self.current_joint_positions.get(joint_name)
            short_name = joint_name.replace(prefix, "")
            label = JOINT_LABELS.get(short_name, short_name)
            if position is None:
                missing.append(label)
                continue
            parts.append(f"{label}={position:+.3f}rad/{math.degrees(position):+.0f}deg")

        if not parts:
            return "joint angles unavailable; no /joint_states received yet."
        suffix = f"; missing: {', '.join(missing)}" if missing else ""
        return f"joint angles: {', '.join(parts)}{suffix}"

    def format_last_command(self, arm):
        command = self.last_published_commands[arm]
        return (
            "last command: "
            f"y={command['vel_y']:+.2f} m/s, "
            f"z={command['vel_z']:+.2f} m/s, "
            f"yaw={command['yaw']:+.2f} rad/s"
        )

    # ------------------------------------------------------------------ #
    # Servo service helpers
    # ------------------------------------------------------------------ #

    def request_servo_service(self, arm, service_name):
        key = (arm, service_name)
        client = self.servo_clients[arm][service_name]
        if key in self.requested_servo_services or not client.service_is_ready():
            return False
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done, a=arm, s=service_name: self.handle_servo_service_response(a, s, done)
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
                f"{arm.capitalize()} Servo {service_name} failed: {response.message}"
            )
            return
        if service_name == "start":
            self.started_servos.add(arm)
            self.paused_servos.discard(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo started.")
        elif service_name == "pause":
            self.paused_servos.add(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo paused.")
        elif service_name == "unpause":
            self.paused_servos.discard(arm)
            self.get_logger().info(f"{arm.capitalize()} Servo unpaused.")

    def ensure_servo_active(self, arm):
        if arm not in self.started_servos:
            self.request_servo_service(arm, "start")
        elif arm in self.paused_servos:
            self.request_servo_service(arm, "unpause")

    def pause_servo_if_idle(self, arm, now):
        last_seen = self.last_hand_seen_time[arm]
        if arm not in self.started_servos or arm in self.paused_servos:
            return
        if last_seen is None or now - last_seen > self.no_hand_pause_timeout:
            self.publish_twist(arm, 0.0, 0.0, 0.0)
            self.request_servo_service(arm, "pause")
            self.last_operator_palm[arm] = None

    # ------------------------------------------------------------------ #
    # Main control loop
    # ------------------------------------------------------------------ #

    def timer_callback(self):
        now = time.monotonic()
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn("Camera frame was empty; halting.")
            self.publish_halt_commands()
            for arm in self.active_arms:
                self.pause_servo_if_idle(arm, now)
            return

        frame = cv2.flip(frame, 1)
        annotated_frame, results = self.tracker.process_frame(frame)

        commands = {
            arm: {"vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "gripper": None, "seen": False}
            for arm in ("left", "right")
        }

        if results.multi_hand_landmarks and results.multi_handedness:
            assignments = self.assign_hands(results.multi_hand_landmarks, results.multi_handedness)
            for arm, hand in assignments.items():
                if arm not in self.active_arms:
                    continue
                if self.safety_latches[arm] is not None:
                    self.last_hand_seen_time[arm] = now
                    continue
                if not self.accept_operator_hand(arm, hand, now):
                    continue
                commands[arm] = self.hand_to_command(arm, hand)
                commands[arm]["seen"] = True
                self.last_hand_seen_time[arm] = now
                self.last_operator_palm[arm] = palm_center(hand)
                self.ensure_servo_active(arm)

        for arm in self.active_arms:
            if self.safety_latches[arm] is not None:
                ramped = self.ramp_velocities(arm, 0.0, 0.0, 0.0)
                self.publish_twist(arm, **ramped)
                self.release_safety_latch_if_idle(arm, now)
                self.pause_servo_if_idle(arm, now)
                continue

            command = commands[arm]
            if not command["seen"]:
                # Ramp velocity to zero smoothly when hand disappears
                ramped = self.ramp_velocities(arm, 0.0, 0.0, 0.0)
                self.publish_twist(arm, **ramped)
                last_seen = self.last_hand_seen_time[arm]
                if last_seen is None or now - last_seen > self.no_hand_pause_timeout:
                    self.last_operator_palm[arm] = None
                self.pause_servo_if_idle(arm, now)
                continue

            ramped = self.ramp_velocities(arm, command["vel_y"], command["vel_z"], command["yaw"])
            self.publish_twist(arm, **ramped)
            # Keep command dict up to date with actual published velocity for the overlay
            command.update(ramped)

            if command["gripper"] is not None:
                self.publish_gripper(arm, command["gripper"])
            self.publish_target_pose(arm, ramped["vel_y"], ramped["vel_z"], ramped["yaw"])

        self.log_commands(commands)

        if self.show_debug_image:
            self.draw_debug_overlay(annotated_frame, commands)
            cv2.imshow('Hand Teleop', annotated_frame)
            cv2.waitKey(1)

    # ------------------------------------------------------------------ #
    # Hand assignment — use MediaPipe handedness instead of x-position
    # ------------------------------------------------------------------ #

    def assign_hands(self, hand_landmarks_list, handedness_list):
        """
        Use MediaPipe's own Left/Right label (mirrored because we flip the frame).
        MediaPipe classifies from the *camera* perspective; after cv2.flip(frame,1)
        its 'Right' label corresponds to the operator's left hand, and vice-versa.
        """
        assignments = {}
        for hand_landmarks, handedness in zip(hand_landmarks_list, handedness_list):
            # MediaPipe label after horizontal flip:  Right→left arm, Left→right arm
            mp_label = handedness.classification[0].label  # "Left" or "Right"
            arm = "left" if mp_label == "Right" else "right"

            if arm in assignments:
                # Duplicate label (rare) — fall back to x-position heuristic
                px, _ = palm_center(hand_landmarks)
                arm = "left" if px < 0.5 else "right"

            if arm not in self.active_arms:
                continue
            assignments[arm] = hand_landmarks

        return assignments

    def accept_operator_hand(self, arm, hand, now):
        if not self.operator_lock_enabled:
            return True

        px, py = palm_center(hand)
        last_seen = self.last_hand_seen_time[arm]
        last_palm = self.last_operator_palm[arm]

        if last_seen is None or last_palm is None or now - last_seen > self.no_hand_pause_timeout:
            local_x, local_y = self.control_coordinates(arm, px, py)
            distance_from_center = math.hypot(local_x - 0.5, local_y - 0.5)
            if distance_from_center > self.operator_acquire_radius:
                self.log_operator_lock(
                    arm,
                    "ignoring hand outside acquisition zone; move the operator hand near its pane centre to take control",
                )
                return False
            return True

        jump = math.hypot(px - last_palm[0], py - last_palm[1])
        if jump > self.operator_max_jump:
            self.log_operator_lock(
                arm,
                f"ignoring sudden hand jump ({jump:.2f}); possible second operator or tracking swap",
            )
            return False
        return True

    def log_operator_lock(self, arm, message):
        now = time.monotonic()
        if now - self.last_operator_lock_log_time[arm] < self.operator_lock_log_period:
            return
        self.last_operator_lock_log_time[arm] = now
        self.get_logger().warn(f"{arm.capitalize()} operator lock: {message}.")

    # ------------------------------------------------------------------ #
    # Gesture → velocity mapping
    # ------------------------------------------------------------------ #

    def control_coordinates(self, arm, raw_x, raw_y):
        if not self.split_control_window:
            return raw_x, raw_y

        pane = self.control_pane_for_arm(arm)
        if pane == "left":
            local_x = raw_x * 2.0
        else:
            local_x = (raw_x - 0.5) * 2.0
        return self.clamp(local_x, 0.0, 1.0), raw_y

    def control_pane_for_arm(self, arm):
        if not self.swap_control_panes:
            return arm
        return "right" if arm == "left" else "left"

    def hand_to_command(self, arm, hand):
        # Use palm centre (average of wrist + 4 MCP knuckles) — far more stable
        # than the index fingertip which jitters several pixels per frame.
        raw_x, raw_y = palm_center(hand)

        smooth_x = self.filters[arm]["x"].update(raw_x)
        smooth_y = self.filters[arm]["y"].update(raw_y)
        control_x, control_y = self.control_coordinates(arm, smooth_x, smooth_y)

        # Displacement from the arm's control-pane centre → servo axis.
        # Default lateral mapping is front-view intuitive: hand right -> robot right.
        lateral_error = control_x - 0.5 if self.invert_lateral_axis else 0.5 - control_x
        y_axis = self.normalized_axis(lateral_error, full_scale=self.motion_full_scale)
        z_axis = self.normalized_axis(0.5 - control_y, full_scale=self.motion_full_scale)
        vel_y = y_axis * self.max_linear_speed
        vel_z = z_axis * self.max_linear_speed

        # Roll: angle between index MCP (5) and pinky MCP (17)
        index_mcp = hand.landmark[5]
        pinky_mcp = hand.landmark[17]
        raw_roll = pinky_mcp.y - index_mcp.y
        smooth_roll = self.filters[arm]["roll"].update(raw_roll)
        yaw_axis = self.normalized_axis(smooth_roll, deadzone=0.03, full_scale=0.15)
        yaw = yaw_axis * self.max_angular_speed

        # Pinch distance for gripper (thumb tip ↔ index tip, landmarks 4 & 8).
        # Bridge convention: 0.0 = open, 1.0 = closed. A small pinch should
        # close the gripper, while spread fingers should open it.
        thumb_tip = hand.landmark[4]
        index_tip = hand.landmark[8]
        pinch = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)
        gripper = 1.0 - self.clamp((pinch - 0.03) / 0.15, 0.0, 1.0)

        return {"vel_y": vel_y, "vel_z": vel_z, "yaw": yaw, "gripper": gripper}

    # ------------------------------------------------------------------ #
    # Velocity ramping — prevents jerky starts/stops
    # ------------------------------------------------------------------ #

    def ramp_velocities(self, arm, target_y, target_z, target_yaw):
        """
        Limit how fast each velocity component can change per tick.
        ramp_rate is the max fractional change of max_speed per tick:
          max_delta_linear = ramp_rate * max_linear_speed
          max_delta_angular = ramp_rate * max_angular_speed
        """
        max_dv = self.ramp_rate * self.max_linear_speed
        max_dyaw = self.ramp_rate * self.max_angular_speed
        prev = self.prev_vel[arm]

        ramped_y = prev["vel_y"] + self.clamp(target_y - prev["vel_y"], -max_dv, max_dv)
        ramped_z = prev["vel_z"] + self.clamp(target_z - prev["vel_z"], -max_dv, max_dv)
        ramped_yaw = prev["yaw"] + self.clamp(target_yaw - prev["yaw"], -max_dyaw, max_dyaw)

        self.prev_vel[arm] = {"vel_y": ramped_y, "vel_z": ramped_z, "yaw": ramped_yaw}
        return {"vel_y": ramped_y, "vel_z": ramped_z, "yaw": ramped_yaw}

    # ------------------------------------------------------------------ #
    # Signal helpers
    # ------------------------------------------------------------------ #

    def apply_deadzone(self, value, deadzone=None):
        active_dz = self.deadzone if deadzone is None else deadzone
        if abs(value) < active_dz:
            return 0.0
        # Rescale so output is zero at the deadzone boundary (no velocity jump)
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - active_dz) / (1.0 - active_dz)

    def normalized_axis(self, value, deadzone=None, full_scale=None):
        fs = self.motion_full_scale if full_scale is None else full_scale
        value = self.apply_deadzone(value, deadzone)
        if value == 0.0:
            return 0.0
        return self.clamp(value / fs, -1.0, 1.0)

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(maximum, value))

    # ------------------------------------------------------------------ #
    # Publishers
    # ------------------------------------------------------------------ #

    def publish_twist(self, arm, vel_y, vel_z, yaw):
        self.last_published_commands[arm] = {"vel_y": vel_y, "vel_z": vel_z, "yaw": yaw}
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

    def publish_halt_commands(self):
        if not rclpy.ok():
            return
        for arm in self.active_arms:
            self.publish_twist(arm, 0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ #
    # Debug overlay
    # ------------------------------------------------------------------ #

    def log_commands(self, commands):
        now = time.monotonic()
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        left = commands["left"]
        right = commands["right"]
        self.get_logger().info(
            "Teleop "
            f"L(y={left['vel_y']:.2f}, z={left['vel_z']:.2f}, yaw={left['yaw']:.2f}) "
            f"R(y={right['vel_y']:.2f}, z={right['vel_z']:.2f}, yaw={right['yaw']:.2f})"
        )

    def draw_debug_overlay(self, frame, commands):
        height, width = frame.shape[:2]
        # Per-arm control panes. Each pane has its own centre/deadzone.
        panes = self.control_panes(width, height)
        for arm, (x0, x1, cy) in panes.items():
            if arm not in self.active_arms:
                continue
            cx = (x0 + x1) // 2
            pane_w = x1 - x0
            cv2.rectangle(frame, (x0, 0), (x1 - 1, height - 1), (45, 45, 45), 1)
            cv2.line(frame, (cx, 0), (cx, height), (80, 80, 80), 1)
            cv2.line(frame, (x0, cy), (x1, cy), (80, 80, 80), 1)
            dz_px = int(self.deadzone * pane_w)
            fs_px = int(self.motion_full_scale * pane_w)
            cv2.circle(frame, (cx, cy), dz_px, (0, 255, 255), 1)
            cv2.circle(frame, (cx, cy), fs_px, (0, 180, 255), 1)
            cv2.putText(
                frame,
                arm.upper(),
                (x0 + 12, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (200, 200, 200),
                2,
            )

        y_text = 28
        for arm in self.active_arms:
            cmd = commands[arm]
            color = (0, 220, 0) if cmd["seen"] else (120, 120, 120)
            gripper_str = f" grip={cmd['gripper']:.2f}" if cmd["gripper"] is not None else ""
            text = (
                f"{arm}: y={cmd['vel_y']:.2f} "
                f"z={cmd['vel_z']:.2f} yaw={cmd['yaw']:.2f}{gripper_str}"
            )
            cv2.putText(frame, text, (12, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            y_text += 24

        # Speed-bar indicator per arm
        bar_w = 120
        for i, arm in enumerate(self.active_arms):
            cmd = commands[arm]
            speed = math.hypot(cmd["vel_y"], cmd["vel_z"])
            frac = min(speed / self.max_linear_speed, 1.0)
            bar_x = 12 + i * (bar_w + 8)
            bar_y = height - 20
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 10), (60, 60, 60), -1)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(frac * bar_w), bar_y + 10),
                          (0, 220, 0) if cmd["seen"] else (80, 80, 80), -1)
            cv2.putText(frame, arm[0].upper(), (bar_x - 14, bar_y + 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    def control_panes(self, width, height):
        cy = height // 2
        if not self.split_control_window:
            return {
                "left": (0, width, cy),
                "right": (0, width, cy),
            }
        mid = width // 2
        physical_panes = {
            "left": (0, mid, cy),
            "right": (mid, width, cy),
        }
        return {
            arm: physical_panes[self.control_pane_for_arm(arm)]
            for arm in ("left", "right")
        }


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_halt_commands()
        except Exception as exc:
            node.get_logger().debug(f"Skipping halt publish during shutdown: {exc}")
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
