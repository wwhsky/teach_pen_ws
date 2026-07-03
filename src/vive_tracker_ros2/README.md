# vive_tracker_ros2

ROS 2 nodes for reading HTC Vive Tracker poses and buttons.

For this project, SteamVR/OpenVR is the preferred pose source because it has
tested better for tracking stability:

```text
Tracker -> USB Dongle -> SteamVR/OpenVR -> ROS 2 Pose/TF
```

The package also contains a `libsurvive` node. It is useful for direct button
packets, Lighthouse visibility debugging, and pose comparison:

```text
Tracker -> USB Dongle -> libsurvive -> ROS 2 Pose/Joy/TF
```

Current `vive_tracker.launch.py` starts the `libsurvive` node. Run
`vive_tracker_node` directly when using the preferred SteamVR/OpenVR pose path.

## Scheme comparison

Both implementations are kept intentionally.

SteamVR/OpenVR requires SteamVR to be installed and running. In a tracker-only
setup without an HMD, SteamVR also needs null-driver configuration. It is heavier
and less open, but in current tests its pose fusion is better: stationary poses
are more stable, and stopping after motion has less overshoot.

`libsurvive` is open source and talks to the Dongle directly. It does not need
SteamVR/OpenVR and exposes more low-level data, including Tracker buttons,
Lighthouse visibility, and raw optical observations. In current tests, however,
its pose can drift while stationary and overshoot after motion, likely related
to optical/IMU fusion and filtering. For now, use SteamVR/OpenVR for the main
pose path, and use `libsurvive` for buttons, raw data, visibility debugging, and
algorithm comparison.

## Dependencies

Build the workspace-local `libsurvive`:

```bash
sudo apt install build-essential cmake libusb-1.0-0-dev libjson-c-dev libeigen3-dev

cd ~/code/teach_pen_ws
git clone https://github.com/cntools/libsurvive.git third_party/libsurvive
touch third_party/libsurvive/COLCON_IGNORE
cmake -S third_party/libsurvive \
  -B third_party/libsurvive/build-local \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_OPENBLAS=OFF \
  -DUSE_OPENCV=OFF \
  -DBUILD_GATT_SUPPORT=OFF
cmake --build third_party/libsurvive/build-local --parallel
colcon build --packages-select vive_tracker_ros2
```

If direct USB access requires `sudo`, install the included udev rule:

```bash
sudo cp third_party/libsurvive/useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

OpenVR is optional. If its headers and library are available, the legacy
SteamVR node is built too:

```bash
sudo apt install libopenvr-dev libopenvr-api1
```

## Run with SteamVR/OpenVR

Start SteamVR first, then run:

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run vive_tracker_ros2 vive_tracker_node --ros-args \
  -p tracking_universe:=raw
```

Use `tracking_universe:=raw` so SteamVR Room Setup does not add an extra room
origin transform. The project calibration then solves `robot_base -> steamvr_base`.

## Run with libsurvive

Completely exit SteamVR because it competes with `libsurvive` for the Dongle.
For two Lighthouse 1.0 base stations, use `b/c` mode.

First calibration:

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py force_calibrate:=true
```

Keep the Tracker stationary and visible to both base stations until
`MPFIT success` appears. Normal startup after calibration:

```bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py
```

The node publishes:

```text
/vive_tracker/pose
/vive_tracker/<serial>/pose
/vive_tracker/buttons
/vive_tracker/<serial>/buttons
/vive_tracker/visibility
/tf: steamvr_base -> tracker_frame
```

`/vive_tracker/visibility` publishes:

```text
[visible_lighthouses, total_measurements, lh0_x, lh0_y, lh1_x, lh1_y]
```

The final four values are recent valid optical measurements for each
Lighthouse and scan axis.

The button topics use `sensor_msgs/msg/Joy`:

```text
buttons[0] = trigger
buttons[1] = grip
buttons[2] = thumb/trackpad
buttons[3] = menu

axes[0] = trackpad x
axes[1] = trackpad y
axes[2] = trigger raw
```

Button events come directly from Watchman/Dongle packets and do not require a
valid optical pose.

## Parameters

```text
frame_id                 default: steamvr_base
child_frame_id           default: tracker_frame
topic_prefix             default: /vive_tracker
device_serial            default: empty, first Tracker is primary
publish_tf               default: true
publish_first_pose_topic default: true
debug_events             default: false
lighthouse_count         default: 2
lighthouse_generation    default: 1
center_on_lighthouse     default: true
force_calibrate          default: false
use_raw_observation      default: false, publish raw MPFIT optical observations
libsurvive_verbosity     default: 1
poll_rate_hz             default: 250.0
config_file              default: empty, use libsurvive XDG config
```

Useful commands:

```bash
ros2 topic echo /vive_tracker/pose
ros2 topic echo /vive_tracker/buttons
ros2 topic echo /vive_tracker/visibility
ros2 run tf2_ros tf2_echo steamvr_base tracker_frame
```

Use raw optical observations for calibration or precise point sampling:

```bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py use_raw_observation:=true
```

Raw observations avoid IMU integration drift, but update less smoothly and stop
when there is no valid Lighthouse solution.

Run the retained OpenVR implementation directly:

```bash
ros2 run vive_tracker_ros2 vive_tracker_node
```
