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
        DeclareLaunchArgument("tracking_universe", default_value="standing"),
        DeclareLaunchArgument("update_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("publish_first_pose_topic", default_value="true"),
        DeclareLaunchArgument("debug_devices", default_value="false"),
        Node(
            package="vive_tracker_ros2",
            executable="vive_tracker_node",
            name="vive_tracker_node",
            output="screen",
            parameters=[{
                "frame_id": LaunchConfiguration("frame_id"),
                "child_frame_id": LaunchConfiguration("child_frame_id"),
                "topic_prefix": LaunchConfiguration("topic_prefix"),
                "device_serial": LaunchConfiguration("device_serial"),
                "tracking_universe": LaunchConfiguration("tracking_universe"),
                "update_rate_hz": LaunchConfiguration("update_rate_hz"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "publish_first_pose_topic": LaunchConfiguration("publish_first_pose_topic"),
                "debug_devices": LaunchConfiguration("debug_devices"),
            }],
        ),
    ])
