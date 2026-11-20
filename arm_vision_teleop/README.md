# arm_vision_teleop

Teleoperate the SO-101 with your **hand in front of the laptop webcam**. MediaPipe
tracks 21 hand landmarks in each frame. Moving your hand moves the gripper tip,
pinching closes the gripper, and making a fist pauses.

```
 Windows                         WSL2 (ROS 2 Humble)
┌──────────────┐  JPEG/TCP  ┌───────────────┐ /camera/image_raw ┌──────────────┐
│cam_streamer.py├──:5005───>│ camera_bridge ├──────────────────>│ hand_tracker │ MediaPipe
└──────────────┘            └───────────────┘                   └──────┬───────┘
                                         /hand/position, /hand/openness, /hand/engaged
                                                                       v
           RViz <─ robot_state_publisher <─┬─ /joint_states ── ┌───────────────┐
                                           │                   │ hand_follower │ IK (placo)
           SO-101 <─ follower_bridge <─────┘   (real:=true)    └───────────────┘
```

Why the Windows step: the WSL2 kernel has no camera drivers (`CONFIG_MEDIA_SUPPORT` is off),
so the webcam can't be attached to WSL, even with usbipd. On native Ubuntu, skip it and
use `camera_source:=local`.

## One-time setup

**WSL** (already done on this machine):
```bash
cd ~/RobotArm
python3 -m venv .venv_vision
.venv_vision/bin/pip install mediapipe opencv-python-headless
colcon build --packages-select arm_joy_teleop arm_vision_teleop
```
MediaPipe and OpenCV live in their own venv, the same way `lerobot_real` uses the lerobot
conda env. Your `~/.local` numpy 2.x breaks Ubuntu's `cv2`/`cv_bridge` in system Python,
and the venv sidesteps that. The launch file runs `camera_bridge` and `hand_tracker` with
the venv's Python. `hand_follower` runs on system Python (it needs `placo`).
The hand model (`hand_landmarker.task`, 7.8 MB) is cached in `~/.cache/arm_vision_teleop/`
and downloaded automatically if it's missing.

**Windows** (PowerShell, once):
```powershell
py -m pip install opencv-python
```

## Run

```bash
# WSL
source ~/RobotArm/install/setup.bash
ros2 launch arm_vision_teleop hand_teleop.launch.py              # sim only
ros2 launch arm_vision_teleop hand_teleop.launch.py real:=true   # + real arm
```
```powershell
# Windows: streams the webcam to WSL (WSL2 forwards localhost automatically)
py \\wsl.localhost\Ubuntu-22.04\home\kirupa_krishan\RobotArm\src\arm_vision_teleop\windows\cam_streamer.py
```
(Or copy `cam_streamer.py` somewhere on Windows. Options: `--device 1`, `--preview`, `--port`.)

A preview window shows the camera with your hand landmarks, the status
(ENGAGED / PAUSED / NO HAND), the gripper opening, and along the bottom
**fps / tracking time / latency**. The tracker draws the window itself, so frames
don't go through DDS. `/hand/debug_image` is still published if something subscribes
to it (e.g. `rqt_image_view`), but that's slow on WSL because each raw frame is ~900 KB.
Launch args: `real`, `dry_run`, `rviz`, `image_view`, `camera_source`, `port`, `arm_port`,
`robot_id`, `params_file`.

## How to control it

| Gesture | Effect |
|---|---|
| Open hand in view | **Engaged.** The tip follows your hand from where it was when you engaged. |
| Move hand left / right | tip moves right / left (robot y) |
| Move hand up / down | tip moves up / down (z) |
| Move hand toward / away from the camera | tip moves forward / back (x) |
| Thumb–index distance | gripper opening (pinch = closed, spread = open) |
| Fist, or hand out of view | **Paused.** The arm holds; reposition your hand, then open it again. |

This works like lifting a mouse to re-centre it: make a fist, move your hand back to the
middle of the frame, open it, and carry on. Wrist flex and roll hold their angles.
The defaults assume you sit **behind the robot, facing the same way**, with the camera
in front of you. If an axis feels backwards, flip its `*_sign` in `config/hand_teleop.yaml`.
To make it more or less sensitive, change `gain_lateral`, `gain_vertical` and `gain_depth`.

## Arm mode: your elbow is the base (`mode:=arm`)

```bash
ros2 launch arm_vision_teleop hand_teleop.launch.py mode:=arm
```

This adds MediaPipe **Pose** (shoulder/elbow/wrist) on top of the hand model and maps your
arm onto the robot **absolutely**: the robot's pose always matches your arm's.

| You | Robot |
|---|---|
| Forearm direction (elbow → wrist): swing left/right, raise/lower, point at the camera | where the gripper tip points from the robot's shoulder (`arm.anchor_m`) |
| Elbow bend (shoulder–elbow–wrist): straight = far, bent = near | how far the tip reaches (`reach_min_m` … `reach_max_m`) |
| Hand bent at the wrist vs your forearm | `wrist_flex` |
| Palm rotation about the forearm (palm facing the camera = 0) | `wrist_roll` |
| Thumb–index pinch | gripper |
| Fist, or elbow/hand out of view | pause (arm holds) |

A rigid forearm pivoting at a fixed elbow can only point in a direction (2 DOF), so the
third DOF, reach, comes from the elbow bend, which needs your **shoulder** in view. If the
shoulder isn't visible, the reach falls back to `reach_fixed_m`, or you can set
`arm.reach_source: fixed` to always use it. Coming back from a pause, the arm *glides* to
match your arm (speed-limited); it doesn't snap.

**Framing:** a laptop camera usually crops your elbows. Tilt the screen back or sit back
until the window shows the blue upper-arm/forearm lines with a red dot on your elbow.
The overlay prints the elbow angle, wrist flex and wrist roll it measures. If it says
`ELBOW NOT IN VIEW`, the elbow is out of frame or hidden. If a robot joint moves
backwards relative to yours, flip `arm.wrist_flex_sign` / `arm.wrist_roll_sign`, or the
sign of an entry in `arm.axis_map`.

Topic: `/arm/pose` (`sensor_msgs/JointState` used as named values: forearm_x/y/z,
elbow_angle, wrist_flex, wrist_roll).

## Safety

- It only moves while an open hand is tracked. A fist, a lost hand or stale data
  (`hand_timeout_s`, 0.3 s) pauses immediately.
- The tip is kept inside `workspace_min/max` (the default keeps it above the table),
  joint speeds and tip speed are capped, and the URDF joint limits are enforced.
- `max_lead_m` stops the target from running far ahead when you reach past the arm's range,
  so the arm doesn't lurch when your hand comes back.
- With `real:=true`, it starts from the arm's measured pose, `follower_bridge` adds its own
  90°/s cap, and Ctrl+C moves the arm to rest and then turns torque off.
- Try `real:=true dry_run:=true` first (torque off), and keep
  `ros2 run lerobot_real torque_off` handy.

## Troubleshooting

- *Streamer says "Waiting for camera_bridge"*: start the ROS launch first. Check that
  `localhostForwarding` isn't disabled in `%UserProfile%\.wslconfig`; if it is, use
  `--host <WSL IP>` (`hostname -I` in WSL).
- *"Cannot open camera"*: close Teams, Zoom or the Camera app, or try `--device 1`.
- *Hand flickers between ENGAGED and PAUSED*: improve the lighting, or raise
  `engage_debounce_frames`, or lower `curl_ratio`.
- *Gripper never fully closes or opens*: adjust `pinch_closed` / `pinch_open`.
- *Preview is slow*: read the bottom line of the window. Low **fps** means the camera
  or streamer is the bottleneck (compare with the fps that `cam_streamer.py` prints;
  laptop webcams drop to ~10 fps in dim light, so add light). A high **track** time
  means the CPU is busy; close RViz with `rviz:=false` to check. A high **latency**
  with normal fps means frames are queueing somewhere.
