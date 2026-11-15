# lerobot_real

Connects a real **SO-101 (LeRobot) arm** and the RViz simulation, in either direction:

| Mode | Launch | Node | Data flow |
|---|---|---|---|
| sim → real | `sim_to_real.launch.py` | `follower_bridge` | sliders / IK → `/joint_states` → motors |
| real → sim | `real_to_sim.launch.py` | `real_to_sim` | motors (moved by hand) → `/joint_states` → RViz |

Run only one mode at a time: both use the serial port and `/joint_states`.

### sim → real

```
joint_state_publisher_gui ─┐
  (or arm_movement IK)     ├─ /joint_states ─┬─> robot_state_publisher ─> RViz
                           │                 └─> follower_bridge ─> SO-101 (USB)
                           │                         │
                           └─────── /lerobot/joint_states <── measured pose
```

## Build

```bash
cd ~/RobotArm && colcon build --packages-select arm_movement lerobot_real
source install/setup.bash
```

LeRobot is **not** installed into the system Python; the launch files run the
node with `~/miniforge3/envs/lerobot/bin/python` (override with
`lerobot_env:=/path/to/env`). Don't activate conda when building or launching.

## Run

```bash
# 1. Check the mapping first: torque OFF, move the real arm by hand and compare
ros2 launch lerobot_real sim_to_real.launch.py dry_run:=true
ros2 topic echo /lerobot/joint_states

# 2. Follow the sliders in joint_state_publisher_gui
ros2 launch lerobot_real sim_to_real.launch.py

# Bridge only (e.g. with arm_move.launch.py's differential IK as the source)
ros2 launch lerobot_real follower_bridge.launch.py
```

Before step 2, set the GUI sliders close to the arm's real pose; the bridge ramps
from the measured pose at `max_velocity_deg_s` (default 90°/s) anyway.

### real → sim (RViz mirrors the arm)

```bash
ros2 launch lerobot_real real_to_sim.launch.py   # rviz:=false to skip RViz
```

Turns torque **off** (the arm goes limp, so start with it resting) and publishes the
measured joints on `/joint_states` at 30 Hz. robot_state_publisher + RViz come from
`arm_movement`, **without** `joint_state_publisher_gui`. On Ctrl+C, torque stays off.
The real arm can sit slightly past the URDF limits (e.g. `shoulder_lift` at rest);
set `clamp_to_urdf_limits: true` if you'd rather RViz never shows that.

## Emergency: turn torque off

```bash
ros2 run lerobot_real torque_off             # all motors limp immediately
ros2 run lerobot_real torque_off --delay 3   # 3 s countdown to grab the arm first
ros2 run lerobot_real torque_off --motors gripper
```

Works without conda (re-launches itself with the lerobot env), verifies each motor
reads back torque-off, and refuses to run while `follower_bridge` is still running
(stop the launch first, or pass `--force`). If a motor can't be confirmed, cut arm power.

## Rest pose

When you **Ctrl+C** the launch, the bridge moves the arm slowly (`rest_speed_deg_s`,
default 30°/s, all joints in sync) to the rest pose, then turns torque off.
Press Ctrl+C **again** (after ~1 s) to abort the move; the arm goes limp right away.
Turn this off with `go_to_rest_on_exit: false`.

```bash
ros2 run lerobot_real go_to_rest            # same move, standalone (bridge must be stopped)
ros2 run lerobot_real go_to_rest --hold     # stay powered at rest
ros2 run lerobot_real go_to_rest --record   # place the arm at rest by hand, save it
```

`--record` writes `~/.config/lerobot_real/rest_pose.yaml`, which both the bridge and
`go_to_rest` use. Without it, the defaults in `lerobot_real/rest.py` are used.
The move is a straight line in joint space, so start it from a pose that is clear of the table.

## Safety behaviour

- Doesn't move until the first `/joint_states` message arrives, then ramps from the measured pose.
- Targets are clamped to the URDF limits, and every joint is speed-limited.
- Holds the last pose if `/joint_states` stops for `command_timeout_s`.
- On exit the arm moves to rest, then torque is turned off (`disable_torque_on_exit`). If the rest move is aborted or fails, it goes limp where it is, so **support the arm**.

## Tuning (`config/so101_follower.yaml`)

Settings under `/**` (port, robot id, signs, offsets, gripper range) are shared by both nodes.

If a real joint moves the opposite way to RViz, flip its entry in `joint_signs`.
If it sits a constant angle away, set `joint_offsets_deg`. If the gripper's
open/close range doesn't match, adjust `gripper_closed_rad` / `gripper_open_rad`.
