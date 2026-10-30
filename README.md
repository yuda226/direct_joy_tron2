# direct_joy

Standalone ROS 2 adaptation of the drive portion of
`i2r_sliding_autonomy_control_direct_joystick` in the I2R workspace.
The original package is unchanged. This copy keeps its Logitech X/D stick
mappings, differential/omni-drive option, speed limits, and turbo scaling.
It deliberately replaces A-to-enable/right-trigger-to-disable with hold-to-drive.

There are no I2R, sliding-autonomy, safety-manager, MRCCC, robot-config, speech,
mast, horn, or lighting dependencies. Those ancillary functions are not copied.
Only standard ROS 2 packages are required.

## Build

Use a fresh terminal sourced with the ROS distribution used by this workspace
(Jazzy on this machine), not the I2R overlay:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/yuda/robot/limx_ws
colcon build --base-paths src/direct_joy --packages-select direct_joy --symlink-install
source install/direct_joy/share/direct_joy/local_setup.bash
```

## Launch

Launch the controller and standard `joy_node` together:

```bash
ros2 launch direct_joy direct_joy.launch.py
```

This reads device 0, publishes raw input on `/joy1/joy`, and publishes
`geometry_msgs/msg/Twist` on `/cmd_vel`.
The driver repeats at 20 Hz so holding a stationary stick does not cause a timeout.

Use another output topic or an already-running joystick driver:

```bash
ros2 launch direct_joy direct_joy.launch.py cmd_vel_topic:=/robot/cmd_vel
ros2 launch direct_joy direct_joy.launch.py start_joy:=false joy_topic:=/joy1/joy
```

Run only the velocity converter, without the driver:

```bash
ros2 run direct_joy direct_joy_node --ros-args \
  --params-file /home/yuda/robot/limx_ws/src/direct_joy/config/direct_joy.yaml
```

An external joystick publisher must send periodic input faster than `joy_timeout`.
Configure its `autorepeat_rate` to 20 Hz or greater.

## Controls

1. Center both sticks and release RB to arm the controller.
2. Hold **RB** (right shoulder button, index 5) while moving the sticks.
3. Release RB to publish zero velocity immediately.

| Input | Output |
| --- | --- |
| Left stick up/down | `linear.x`, forward/backward |
| Left stick left/right | `linear.y`, only with `omni_drive: true` |
| Right stick left/right | `angular.z`, yaw |
| Left trigger while holding RB | Turbo |

Default maximum speeds match the reference robot: 1 m/s linear and 1 rad/s yaw.
Normal speed is maximum divided by `turbo_scalar` (2.0 in the supplied YAML,
1.2 when running without a parameter file); turbo uses the maximum.
A, right trigger, and other buttons have no drive-mode switching functions.

## Optional TRON2 mode controls

Start `tron2_control_mode` separately with the correct robot connection settings,
then opt in to joystick service calls:

```bash
ros2 launch direct_joy direct_joy.launch.py mode_controls:=true
```

Mode controls are disabled by default. `mode_service_prefix` defaults to
`/tron2_control_mode`; change it if that node is namespaced or renamed.
The startup parameter `mode_service_timeout` defaults to 5 seconds and must
exceed the mode node's acknowledgment timeout.

| Combination | Service/action |
| --- | --- |
<!-- | LB + X | Unassigned (Idle has no separate service) | -->
| RB + Y | `stand` |
| RB + B | `walk` |
| RB + A | `sitdown` |
| LB + RB + Y | `recover` (hold both shoulders before pressing Y) |
| Y + A held continuously for at least 2 seconds | `stair_mode`, alternating ON/OFF |
| X + D-pad Up | `base_height`, `true` (raise) |
| X + D-pad Down | `base_height`, `false` (lower) |
| Both stick buttons | `emgy_stop`, stop only, never a release toggle |

Each combination sends at most one request until all mode controls are released.
Height requests do not repeat while held; release X before requesting another
step. Begin with mode controls released after startup, a layout change, malformed
input, or joystick timeout. Emergency requests are allowed even before this
neutral initialization and can interrupt a pending local request.
Do not roll between combinations: release first, and press modifiers before the
final action button. Conflicting face-button chords do not select a mode.
RB + Y/B/A are detected before velocity is published, so they request modes
instead of accidentally driving while the enable button is held. LB + Y/B/A
alone do not request a mode.

These are Logitech X/D mappings, not universal controller labels:

| Control | X layout index | D layout index |
| --- | --- | --- |
| A / B / X / Y buttons | 0 / 1 / 2 / 3 | 1 / 2 / 0 / 3 |
| LB / RB buttons | 4 / 5 | 4 / 5 |
| Left / right stick buttons | 9 / 10 | 10 / 11 |
| D-pad vertical axis | 7 | 5 |

Positive D-pad vertical values mean Up. Verify indexes using raw Joy messages
with the robot disconnected before enabling these controls.

Mode controls suppress velocity, including while a request is pending.
Afterward release drive-enable and center both sticks to rearm driving.
An emergency request additionally latches local velocity inhibition until a
recovery request succeeds; this is not confirmation that the robot has physically
recovered. Emergency requests use the same network and firmware as other
commands and are not a substitute for the physical emergency stop.

Stair mode is assumed **OFF at joystick-node startup**. The first long hold
requests ON; only a successful response changes the locally tracked state.
The robot must actually start with stairs off, and no other controller should
change that state. Rejection, missing services, and timeouts are logged, never
automatically retried. After a timeout the robot outcome is unknown: inspect and
resynchronize stair state before using the toggle again.
Service success means accepted, not physically completed. Wait for standing to
physically finish before requesting walk; there is no automatic stand/walk
sequence or physical-mode feedback in this joystick node.

## Configuration and stopping

Edit [config/direct_joy.yaml](config/direct_joy.yaml) before startup, or supply
`params_file:=/absolute/path/to/your.yaml`. Parameters are read at startup.

- `controller_mode`: `auto`, `x`, or `d`. Auto recognizes the original Logitech
  X layout (8 axes/11 buttons) and D layout (6 axes/12 buttons).
  Explicit modes allow additional buttons/axes, but retain Logitech mappings.
- `enable_button`: hold-to-drive button index (default 5).
- `max_speed_linear_x`, `max_speed_linear_y`, `max_speed_rotation`: positive limits.
- `turbo_scalar`: at least 1.0; normal speed divisor.
- `omni_drive`: false by default; forces `linear.y` to zero.
- `deadzone`: centered-stick tolerance and motion deadband (default 0.05).
- `joy_timeout`: positive stale-input timeout in seconds (default 0.5).
- `publish_rate`: heartbeat rate, 1-1000 Hz (default 20).

Malformed input or a controller layout change clears the command. Stale input
clears it at the first timer tick after the timeout (nominally within 0.55 s
of the last input with the defaults). Driving can resume only after releasing
enable and centering both sticks. The timeout uses a monotonic clock, independent
of ROS simulation time.

The converter publishes continuously, including zero commands when disabled.
**Do not share its output directly with another command source:** use a velocity
multiplexer if multiple sources are needed.

**This package does not provide obstacle avoidance, an emergency stop, or the
I2R safety pipeline.** Retain the robot's own safety controls and base-controller
command watchdog. A killed process cannot keep publishing zero; the base must
stop when velocity commands cease. Verify controller mappings and speeds with
the base disabled before allowing physical movement. Hardware operation has not
been validated by the package tests.

## Test

```bash
colcon test --base-paths src/direct_joy --packages-select direct_joy \
  --event-handlers console_direct+
colcon test-result --test-result-base build/direct_joy --verbose
```

Tests cover X/D mappings, hold-to-drive, centering, release, turbo, differential
and omni-drive, timeout/rearming, malformed input, and parameter validation.
The launch integration test uses ROS domain 191 and unique test topics, with the
physical joystick driver disabled. It never publishes to robot command topics.
