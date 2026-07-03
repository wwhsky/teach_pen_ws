from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def find_default_calibration_config_dir():
    candidates = [Path.cwd() / "config" / "calibration"]
    candidates.extend(
        parent / "config" / "calibration"
        for parent in Path(__file__).resolve().parents
    )

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return candidates[0]


def include_moveit_launch(moveit_config_package, launch_file, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                "launch",
                launch_file,
            ])
        ),
        condition=condition,
    )


def generate_launch_description():
    default_calibration_config_dir = find_default_calibration_config_dir()
    calibration_files = [
        str(default_calibration_config_dir / "teaching_pen_tip.yaml"),
        str(default_calibration_config_dir / "welding_torch_tip.yaml"),
        str(default_calibration_config_dir / "vr_to_robot.yaml"),
        str(default_calibration_config_dir / "workpiece.yaml"),
    ]

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_calibration_tf = LaunchConfiguration("start_calibration_tf")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="zu5"),
        DeclareLaunchArgument("moveit_config_package", default_value="jaka_zu5_moveit_config"),
        DeclareLaunchArgument("start_gazebo", default_value="true"),
        DeclareLaunchArgument("start_calibration_tf", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("teach_pen"),
                "rviz",
                "real_robot.rviz",
            ]),
        ),

        Node(
            package="calibration",
            executable="publish_calibration_tf",
            name="publish_calibration_tf",
            output="screen",
            parameters=[{
                "files": calibration_files,
            }],
            condition=IfCondition(start_calibration_tf),
        ),

        # This starts Ignition Gazebo, robot_state_publisher, move_group,
        # spawns the robot, and spawns the ros2_control trajectory controller.
        include_moveit_launch(
            moveit_config_package,
            "gazebo.launch.py",
            IfCondition(start_gazebo),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="log",
            arguments=["-d", rviz_config],
            condition=IfCondition(start_rviz),
        ),
    ])
