"""
ready_pose_node.py
==================
Publishes a single JointTrajectory to each arm controller to move both arms
to a safe "ready" pose on startup.  Servo is only started *after* this motion
has had time to complete (see servo.launch.py TimerAction delay).

Why direct controller commands instead of MoveIt plan-and-execute?
  - Simpler: does not require move_group to be fully initialised first.
  - Faster startup: no planning latency.
  - The pose is a known-good configuration, so no planning is needed.

The ready pose matches initial_positions.yaml so the robot snaps back to a
predictable state rather than wherever the operator left it last session.
"""

import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── Ready pose ──────────────────────────────────────────────────────────────
# Arms spread outward (pan ±0.4), elbows well-bent (1.8 rad), wrists at 90°.
# This pose is far from all three UR5e singularity types:
#   • Elbow singularity   (elbow ≈ 0 or ±π) → elbow = 1.8 rad  ✓
#   • Shoulder singularity (wrist centre over base axis) → lift = -1.2 ✓
#   • Wrist singularity   (wrist_2 ≈ 0)     → wrist_2 = ±1.57 ✓
# ─────────────────────────────────────────────────────────────────────────────
READY_POSE = {
    "left": {
        "joints": [
            "left_shoulder_pan_joint",
            "left_shoulder_lift_joint",
            "left_elbow_joint",
            "left_wrist_1_joint",
            "left_wrist_2_joint",
            "left_wrist_3_joint",
        ],
        "positions": [0.4, -1.2, 1.8, -2.2, -1.57, 0.0],
        "topic": "/left_arm_controller/joint_trajectory",
    },
    "right": {
        "joints": [
            "right_shoulder_pan_joint",
            "right_shoulder_lift_joint",
            "right_elbow_joint",
            "right_wrist_1_joint",
            "right_wrist_2_joint",
            "right_wrist_3_joint",
        ],
        "positions": [-0.4, -1.2, 1.8, -2.2, 1.57, 0.0],
        "topic": "/right_arm_controller/joint_trajectory",
    },
}

# How long (seconds) the robot has to reach the ready pose.
# Set conservatively: the motion completes in ~2 s, we wait 3.5 s total.
MOTION_DURATION_SEC = 3.5


class ReadyPoseNode(Node):
    """Publish ready-pose JointTrajectory goals to both arm controllers."""

    def __init__(self):
        super().__init__("ready_pose_node")
        # Named traj_publishers to avoid shadowing Node.publishers (read-only property)
        self.traj_publishers = {
            arm: self.create_publisher(JointTrajectory, cfg["topic"], 10)
            for arm, cfg in READY_POSE.items()
        }
        self.done = False
        # Small delay before publishing so controllers have time to spin up
        self._startup_timer = self.create_timer(1.5, self._send_goals)
        self.get_logger().info(
            "ReadyPoseNode waiting 1.5 s for controllers, then commanding ready pose."
        )

    def _send_goals(self):
        self.destroy_timer(self._startup_timer)

        for arm, cfg in READY_POSE.items():
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = cfg["joints"]

            pt = JointTrajectoryPoint()
            pt.positions = cfg["positions"]
            pt.time_from_start.sec = int(MOTION_DURATION_SEC)
            pt.time_from_start.nanosec = int(
                (MOTION_DURATION_SEC - int(MOTION_DURATION_SEC)) * 1e9
            )
            msg.points.append(pt)

            self.traj_publishers[arm].publish(msg)
            self.get_logger().info(
                f"  {arm.capitalize()} arm → ready pose "
                f"(pan={cfg['positions'][0]:.2f}, "
                f"lift={cfg['positions'][1]:.2f}, "
                f"elbow={cfg['positions'][2]:.2f})"
            )

        self.get_logger().info(
            f"Ready pose commands sent. Servo will start in "
            f"{MOTION_DURATION_SEC + 1.5:.1f} s (after motion completes)."
        )
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = ReadyPoseNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()