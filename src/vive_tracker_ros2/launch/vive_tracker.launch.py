from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument("show_raw_observation", default_value="false"),
        DeclareLaunchArgument("use_kalman", default_value="true"),
        DeclareLaunchArgument("globalscenesolver", default_value="1"),
        DeclareLaunchArgument("use_stationary_sensor_window", default_value="1"),
        DeclareLaunchArgument("poser", default_value=""),
        DeclareLaunchArgument("precise", default_value="false"),
        DeclareLaunchArgument("required_meas", default_value="8"),
        DeclareLaunchArgument("time_window_ms", default_value="0.0"),
        DeclareLaunchArgument("syncs_per_run", default_value="1"),
        DeclareLaunchArgument("min_report_time_ms", default_value="-1.0"),
        DeclareLaunchArgument("pose_filter_alpha", default_value="1.0"),
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
                "show_raw_observation": LaunchConfiguration("show_raw_observation"),
                "use_kalman": LaunchConfiguration("use_kalman"),
                "globalscenesolver": LaunchConfiguration("globalscenesolver"),
                "use_stationary_sensor_window": LaunchConfiguration("use_stationary_sensor_window"),
                "poser": LaunchConfiguration("poser"),
                "precise": LaunchConfiguration("precise"),
                "required_meas": LaunchConfiguration("required_meas"),
                "time_window_ms": ParameterValue(
                    LaunchConfiguration("time_window_ms"), value_type=float),
                "syncs_per_run": LaunchConfiguration("syncs_per_run"),
                "min_report_time_ms": ParameterValue(
                    LaunchConfiguration("min_report_time_ms"), value_type=float),
                "pose_filter_alpha": ParameterValue(
                    LaunchConfiguration("pose_filter_alpha"), value_type=float),
                "libsurvive_verbosity": LaunchConfiguration("libsurvive_verbosity"),
                "poll_rate_hz": ParameterValue(
                    LaunchConfiguration("poll_rate_hz"), value_type=float),
                "config_file": LaunchConfiguration("config_file"),
            }],
        ),
    ])
