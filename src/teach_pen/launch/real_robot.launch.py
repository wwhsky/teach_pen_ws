from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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

    model = LaunchConfiguration("model")
    ip = LaunchConfiguration("ip")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    start_executor = LaunchConfiguration("start_executor")
    start_calibration_tf = LaunchConfiguration("start_calibration_tf")
    start_static_tf = LaunchConfiguration("start_static_tf")
    start_rsp = LaunchConfiguration("start_rsp")
    start_move_group = LaunchConfiguration("start_move_group")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    auto_power_on = LaunchConfiguration("auto_power_on")
    auto_enable = LaunchConfiguration("auto_enable")

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="zu5"),
        DeclareLaunchArgument("ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("moveit_config_package", default_value="jaka_zu5_moveit_config"),
        DeclareLaunchArgument("start_executor", default_value="true"),
        DeclareLaunchArgument("start_calibration_tf", default_value="true"),
        DeclareLaunchArgument("start_static_tf", default_value="true"),
        DeclareLaunchArgument("start_rsp", default_value="true"),
        DeclareLaunchArgument("start_move_group", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("teach_pen"),
                "rviz",
                "real_robot.rviz",
            ]),
        ),
        DeclareLaunchArgument("auto_power_on", default_value="true"),
        DeclareLaunchArgument("auto_enable", default_value="true"),

        Node(
            package="teach_pen",
            executable="jaka_trajectory_executor_node",
            name="jaka_trajectory_executor_node",
            output="screen",
            parameters=[{
                "ip": ip,
                "model": model,
                "auto_power_on": auto_power_on,
                "auto_enable": auto_enable,
            }],
            condition=IfCondition(start_executor),
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

        include_moveit_launch(
            moveit_config_package,
            "static_virtual_joint_tfs.launch.py",
            IfCondition(start_static_tf),
        ),
        include_moveit_launch(
            moveit_config_package,
            "rsp.launch.py",
            IfCondition(start_rsp),
        ),
        include_moveit_launch(
            moveit_config_package,
            "move_group.launch.py",
            IfCondition(start_move_group),
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
