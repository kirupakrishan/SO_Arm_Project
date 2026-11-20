# arm_joy_teleop

Drive the SO-101 with a gamepad, in RViz and (optionally) on the real arm. It uses
the standard ROS 2 [`joy`](https://index.ros.org/p/joy/) package (`ros-humble-joy`)
to read the controller.

```
gamepad ─> joy_node ─/joy─> joy_teleop ─/joint_states─┬─> robot_state_publisher ─> RViz
                               ▲                      └─> follower_bridge ─> SO-101   (real:=true)
                               └──── /lerobot/joint_states (start from the real pose)
```

## Build & run

```bash
sudo apt install ros-humble-joy          # already installed on this machine
cd ~/RobotArm && colcon build --packages-select arm_joy_teleop
source install/setup.bash

ros2 launch arm_joy_teleop joy_teleop.launch.py                 # sim only
ros2 launch arm_joy_teleop joy_teleop.launch.py real:=true      # + real arm
ros2 launch arm_joy_teleop joy_teleop.launch.py real:=true dry_run:=true  # real, torque off
```

Other launch args: `device_id:=1` (second pad), `start_mode:=cartesian`, `rviz:=false`,
`params_file:=/path/to/my_pad.yaml`, and `port` / `robot_id` / `lerobot_env`, which are
passed through to `lerobot_real`.

With `real:=true`, the teleop waits up to 2 s for `/lerobot/joint_states` and starts from
the arm's **measured** pose, so the arm doesn't jump. Ctrl+C behaves as it does in
`lerobot_real`: the arm moves to its rest pose, then torque turns off.
Don't run `arm_viz.launch.py` / `joint_state_publisher_gui` at the same time,
because they also publish `/joint_states`.

## Controls (Xbox layout, PlayStation equivalents work the same)

**Hold LB (deadman) for anything to move.** Let go and the arm stays where it is.

| Input | Joint mode | Cartesian mode |
|---|---|---|
| Left stick ↔ | shoulder_pan | tip Y |
| Left stick ↕ | shoulder_lift | tip X |
| Right stick ↕ | elbow_flex | tip Z |
| Right stick ↔ | wrist_flex | wrist_flex |
| D-pad ← / → | wrist_roll | wrist_roll |
| B / A | gripper open / close | gripper open / close |
| X (hold) | glide to `home_pose` | glide to `home_pose` |
| D-pad ↑ / ↓ | speed ×1.25 / ×0.8 | speed ×1.25 / ×0.8 |
| START | toggle joint ↔ Cartesian | toggle |

Cartesian mode moves the gripper tip (`gripper_frame_link`) in the world frame using
damped-least-squares differential IK on pan, lift and elbow (same approach as
`arm_movement/inv_diff_kine.py`). It needs `placo`, which is already installed.

## Remapping your controller

Run `ros2 run joy joy_node` and `ros2 topic echo /joy`, then press buttons and move sticks
to see their indices. Edit `config/gamepad.yaml`, or copy it and pass `params_file:=`.
Every joint takes an `axis` + `scale` and/or a `button_pos` / `button_neg` pair (`-1` = unmapped).
If a joint moves the wrong way, flip its `scale` sign.

## WSL2: getting the controller into Linux

WSL can't see USB devices by default. In an **admin PowerShell on Windows**:

```powershell
winget install usbipd
usbipd list                         # find the gamepad's BUSID, e.g. 2-3
usbipd bind --busid 2-3             # once
usbipd attach --wsl --busid 2-3     # every time you plug it in / restart WSL
```

(Do the same for the SO-101's serial adapter if it isn't attached already.)

**Caveat:** the stock WSL kernel on this machine (`5.15.167.4-microsoft-standard-WSL2`) is
built **without** `CONFIG_INPUT_EVDEV` / `CONFIG_INPUT_JOYDEV`, so no `/dev/input/*` appears.
SDL2, which `joy` uses, can sometimes still read PlayStation (DS4/DualSense) pads through
`/dev/hidraw*` (enabled in this kernel). Try it: `ls /dev/hidraw*`, then
`sudo chmod a+rw /dev/hidraw*` and `ros2 run joy joy_enumerate_devices`.
If no device is listed, the reliable fix is a custom WSL kernel with
`CONFIG_INPUT_EVDEV=y`, `CONFIG_INPUT_JOYDEV=y`, `CONFIG_JOYSTICK_XPAD=y` (Xbox) and
`CONFIG_HID_SONY=y` (PlayStation), pointed to by `kernel=` in `%UserProfile%\.wslconfig`.
On native Ubuntu, none of this is needed.

## Safety

- Deadman button, stale-input timeout (`joy_timeout_s`), per-joint speed limits, URDF joint limits.
- On the real arm, `follower_bridge` adds its own speed cap (90°/s), limit clamping and rest-on-exit.
- Start slowly: the default `speed_scale` is 0.5. Keep `ros2 run lerobot_real torque_off` within reach.
