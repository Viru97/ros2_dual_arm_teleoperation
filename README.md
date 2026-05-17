# Teleoperation Challenge - Dual UR5e Workspace

ROS 2 Humble workspace for visualizing and teleoperating a dual-arm UR5e setup with Robotiq 2F-85 grippers in RViz. The stack uses MoveIt 2, `ros2_control`, MoveIt Servo, OpenCV, and MediaPipe hand tracking.

This is an RViz visualization/control stack, not a physics simulator.

## Implemented

- Dual UR5e robot model with two-finger Robotiq grippers.
- MoveIt 2 planning groups for `left_arm`, `right_arm`, `dual_arm`, and grippers.
- Self-collision-aware planning through MoveIt.
- Fake `ros2_control` trajectory controllers for RViz execution.
- `/joint_states` publishing from `joint_state_broadcaster`.
- Two MoveIt Servo nodes:
  - `/left_servo_node` -> `/left_arm_controller/joint_trajectory`
  - `/right_servo_node` -> `/right_arm_controller/joint_trajectory`
- Camera-based hand teleoperation:
  - MediaPipe detects up to two hands.
  - Palm center controls planar end-effector Y/Z velocity.
  - Hand roll controls yaw velocity.
  - Thumb-index pinch controls gripper open/close.
- Gripper bridge:
  - `/teleop/left_gripper_target` -> `/left_gripper_controller/joint_trajectory`
  - `/teleop/right_gripper_target` -> `/right_gripper_controller/joint_trajectory`
- One-command full launch through `run_full_teleop.sh`.

## Safety And Noise Handling

The assignment does not require perfect hand tracking, so the control path includes several stability measures:

- Palm-center tracking uses wrist and MCP landmarks instead of a fingertip, reducing jitter.
- EMA filtering smooths palm position and hand roll.
- Deadzone around the image center gives a stable stop zone.
- Velocity ramping limits acceleration and avoids jerky starts/stops.
- MoveIt Servo checks singularities, self-collision, scene collision, and joint limits.
- Removing the hand pauses Servo after a short timeout.
- Startup ready pose keeps both arms bent and away from common UR5e singularities.
- Operator lock rejects likely second-person hands or MediaPipe identity swaps:
  - a new hand must first acquire control near the image center;
  - sudden palm jumps are ignored and the arm ramps toward stop.

Tunable launch parameters:

```bash
ros2 launch dual_arm_teleop hand_teleop.launch.py \
  arm:=left \
  max_linear_speed:=0.25 \
  filter_alpha:=0.35 \
  operator_lock_enabled:=true \
  operator_acquire_radius:=0.45 \
  operator_max_jump:=0.30
```

## Dependencies

Install ROS 2 Humble, MoveIt 2, and ros2_control:

```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

Install Python packages:

```bash
python3 -m pip install opencv-python mediapipe numpy
```

This workspace includes source copies of:

- `Universal_Robots_ROS2_Description`
- `ros2_robotiq_gripper`

## Build

```bash
cd ~/teleop_challenge_ws
colcon build
source install/setup.bash
```

## Run Everything

```bash
cd ~/teleop_challenge_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./run_full_teleop.sh
```

Useful launch options:

```bash
./run_full_teleop.sh arm:=left
./run_full_teleop.sh arm:=both show_debug_image:=true
./run_full_teleop.sh max_linear_speed:=0.20 max_angular_speed:=0.7
```

The full launch starts:

1. MoveIt demo stack: RViz, `move_group`, `ros2_control`, controllers.
2. MoveIt Servo nodes.
3. Hand teleop node and gripper bridge.

## Manual Run

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_arm_moveit_config demo.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_arm_moveit_config servo.launch.py
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_arm_teleop hand_teleop.launch.py arm:=left
```

Servo starts paused so it does not fight MoveIt plan-and-execute. The hand teleop node starts or unpauses Servo only while an accepted operator hand is detected, and pauses Servo after the hand disappears.

## Controls

- Move palm to image center: stop.
- Move palm left/right/up/down: planar end-effector motion.
- Roll hand: yaw rotation.
- Pinch thumb and index finger: close gripper.
- Spread thumb and index finger: open gripper.
- Remove hand: halt command and pause Servo.

Start with `arm:=left` while tuning, then use `arm:=both` once both arms are behaving safely.

## Real Robot Integration Design

The current workspace uses fake hardware for RViz, but the ROS graph is intentionally shaped like a real robot integration:

- Teleop publishes Cartesian Servo commands as `TwistStamped`:
  - `/left_servo_node/delta_twist_cmds`
  - `/right_servo_node/delta_twist_cmds`
- MoveIt Servo converts those commands into `JointTrajectory` messages:
  - `/left_arm_controller/joint_trajectory`
  - `/right_arm_controller/joint_trajectory`
- Gripper targets are normalized `Float64` messages:
  - `0.0 = open`
  - `1.0 = closed`
- `gripper_bridge_node` converts normalized gripper targets into Robotiq main-knuckle `JointTrajectory` commands.

For real UR arms, replace the fake hardware/controller layer with real UR drivers and controllers while preserving the command interfaces:

1. Connect each UR5e through `ur_robot_driver` or the deployment's real controller stack.
2. Configure the real arm controllers to accept `FollowJointTrajectory` or `JointTrajectory` commands for the same six arm joints.
3. Configure real Robotiq gripper drivers or a hardware bridge for the main gripper command.
4. Keep MoveIt Servo as the real-time command generator, with conservative speed limits.
5. Validate these topics before enabling motion:

```bash
ros2 topic hz /joint_states
ros2 topic echo /left_arm_controller/joint_trajectory
ros2 topic echo /right_arm_controller/joint_trajectory
ros2 topic echo /teleop/left_gripper_target
ros2 topic echo /teleop/right_gripper_target
```

Before running on hardware, add or verify external safety:

- physical emergency stop,
- reduced-speed teach/test mode,
- workspace boundaries,
- collision geometry for the workcell,
- operator dead-man enable or supervised enable switch,
- joint/velocity/acceleration limits matching the real robot.

## Diagnostics

Servo warnings include joint angles and the last published command, for example:

```text
Left Servo status 1: Moving closer to a singularity, decelerating.
joint angles: sh_pan=..., sh_lift=..., elbow=...
last command: y=..., z=..., yaw=...
```

Check runtime rates:

```bash
ros2 param get /robot_state_publisher publish_frequency
ros2 topic hz /joint_states
```

## Version Control

The submission should be under version control. If this workspace is not already a Git repository:

```bash
git init
git add .
git commit -m "Initial dual-arm teleoperation challenge solution"
```

## Open-Source References

This project uses and includes source from:

- Universal Robots ROS 2 Description
- PickNik Robotics `ros2_robotiq_gripper`
- MoveIt 2 / MoveIt Servo
- MediaPipe Hands
- OpenCV
