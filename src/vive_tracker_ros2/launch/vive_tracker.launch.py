from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="steamvr_base"),
        DeclareLaunchArgument("child_frame_id", default_value="tracker_frame"),
        DeclareLaunchArgument("topic_prefix", default_value="/vive_tracker"),
        DeclareLaunchArgument("device_serial", default_value=""),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("publish_first_pose_topic", default_value="true"),
        DeclareLaunchArgument("debug_events", default_value="false"),
        DeclareLaunchArgument("lighthouse_count", default_value="2"),
        DeclareLaunchArgument("lighthouse_generation", default_value="1"),
        DeclareLaunchArgument("center_on_lighthouse", default_value="true"),
        DeclareLaunchArgument("force_calibrate", default_value="false"),
        DeclareLaunchArgument("use_raw_observation", default_value="false"),
        DeclareLaunchArgument("globalscenesolver", default_value="1"),
        DeclareLaunchArgument("use_stationary_sensor_window", default_value="1"),
        DeclareLaunchArgument("poser", default_value=""),
        DeclareLaunchArgument("precise", default_value="false"),
        DeclareLaunchArgument("visibility_tolerance_ms", default_value="0.0"),
        DeclareLaunchArgument("libsurvive_verbosity", default_value="1"),
        DeclareLaunchArgument("poll_rate_hz", default_value="250.0"),
        DeclareLaunchArgument("config_file", default_value=""),
        Node(
            package="vive_tracker_ros2",
            executable="vive_tracker_libsurvive_node",
            name="vive_tracker_node",
            output="screen",
            parameters=[{
                "frame_id": LaunchConfiguration("frame_id"),
                "child_frame_id": LaunchConfiguration("child_frame_id"),
                "topic_prefix": LaunchConfiguration("topic_prefix"),
                "device_serial": LaunchConfiguration("device_serial"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "publish_first_pose_topic": LaunchConfiguration("publish_first_pose_topic"),
                "debug_events": LaunchConfiguration("debug_events"),
                "lighthouse_count": LaunchConfiguration("lighthouse_count"),
                "lighthouse_generation": LaunchConfiguration("lighthouse_generation"),
                "center_on_lighthouse": LaunchConfiguration("center_on_lighthouse"),
                "force_calibrate": LaunchConfiguration("force_calibrate"),
                "use_raw_observation": LaunchConfiguration("use_raw_observation"),
                "globalscenesolver": LaunchConfiguration("globalscenesolver"),
                "use_stationary_sensor_window": LaunchConfiguration("use_stationary_sensor_window"),
                "poser": LaunchConfiguration("poser"),
                "precise": LaunchConfiguration("precise"),
                "visibility_tolerance_ms": LaunchConfiguration("visibility_tolerance_ms"),
                "libsurvive_verbosity": LaunchConfiguration("libsurvive_verbosity"),
                "poll_rate_hz": LaunchConfiguration("poll_rate_hz"),
                "config_file": LaunchConfiguration("config_file"),
            }],
        ),
    ])
