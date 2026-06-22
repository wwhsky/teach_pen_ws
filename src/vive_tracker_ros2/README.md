# vive_tracker_ros2

ROS 2 node for reading HTC Vive Tracker poses from SteamVR/OpenVR and publishing
them as `geometry_msgs/msg/PoseStamped` plus optional TF transforms.

## Dependencies

On Ubuntu, install the OpenVR development package:

```bash
sudo apt-get install libopenvr-dev libopenvr-api1
```

If OpenVR is installed somewhere else, pass its root directory while building:

```bash
colcon build --packages-select vive_tracker_ros2 --cmake-args -DOPENVR_ROOT=/path/to/openvr
```

`OPENVR_ROOT` must contain `openvr.h` under `headers/` or `include/`, and
`libopenvr_api.so` under `lib/`, `lib64/`, `bin/linux64/`, or `lib/linux64/`.

## Run

Start SteamVR first, make sure the Vive Tracker 3.0 is paired and tracking, then:

```bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py
```

The node publishes:

```text
/vive_tracker/pose
/vive_tracker/<serial>/pose
/vive_tracker/buttons
/vive_tracker/<serial>/buttons
/tf: steamvr_base -> tracker_frame
```

`/vive_tracker/pose` is the first valid tracker seen in each polling cycle.
Per-device topics use a sanitized tracker serial number. The TF child frame is
fixed to `tracker_frame` for the current single-tracker setup.

The button topics use `sensor_msgs/msg/Joy`. `buttons` are ordered as:

```text
buttons[0] = trigger
buttons[1] = grip
buttons[2] = trackpad
buttons[3] = menu
```

`axes` are ordered as:

```text
axes[0] = trackpad x
axes[1] = trackpad y
axes[2] = trigger raw
```

Button states are published when the tracker is connected, even if the tracker
pose is not valid yet. This is useful for pogo-pin short tests before the base
stations arrive.

## Useful Parameters

```text
frame_id                 default: steamvr_base
child_frame_id           default: tracker_frame
topic_prefix             default: /vive_tracker
device_serial            default: empty, publish all trackers
tracking_universe        default: standing, options: standing, seated, raw
update_rate_hz           default: 100.0
publish_tf               default: true
publish_first_pose_topic default: true
debug_devices            default: false, print connected OpenVR devices every 2 seconds
```

Example for one known tracker serial:

```bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py device_serial:=LHR-XXXXXXX
```

Debug connected OpenVR devices, even when the tracker pose is not valid yet:

```bash
ros2 launch vive_tracker_ros2 vive_tracker.launch.py tracking_universe:=raw debug_devices:=true
```

Watch button state:

```bash
ros2 topic echo /vive_tracker/buttons
```

Query TF:

```bash
ros2 run tf2_ros tf2_echo steamvr_base tracker_frame
```
