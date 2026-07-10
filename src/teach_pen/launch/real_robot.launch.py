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
        str(default_calibration_config_dir / "camera_hand_eye.yaml"),
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
    start_camera = LaunchConfiguration("start_camera")
    start_seam_perception = LaunchConfiguration("start_seam_perception")
    rviz_config = LaunchConfiguration("rviz_config")
    auto_power_on = LaunchConfiguration("auto_power_on")
    auto_enable = LaunchConfiguration("auto_enable")
    camera_id = LaunchConfiguration("camera_id")
    camera_param_file = LaunchConfiguration("camera_param_file")
    teaching_path_file = LaunchConfiguration("teaching_path_file")
    detected_path_file = LaunchConfiguration("detected_path_file")
    photo_pose_file = LaunchConfiguration("photo_pose_file")
    camera_hand_eye_file = LaunchConfiguration("camera_hand_eye_file")
    detected_path_z_offset = LaunchConfiguration("detected_path_z_offset")

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
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_seam_perception", default_value="true"),
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
        DeclareLaunchArgument("camera_id", default_value="M3GM620B014"),
        DeclareLaunchArgument("camera_param_file", default_value="CameraSettingweld.json"),
        DeclareLaunchArgument("teaching_path_file", default_value="config/paths/demo_path.yaml"),
        DeclareLaunchArgument("detected_path_file", default_value="config/paths/detected_seam_path.yaml"),
        DeclareLaunchArgument("photo_pose_file", default_value="config/paths/photo_pose.yaml"),
        DeclareLaunchArgument("detected_path_z_offset", default_value="-0.003"),
        DeclareLaunchArgument(
            "camera_hand_eye_file",
            default_value=str(default_calibration_config_dir / "camera_hand_eye.yaml"),
        ),

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

        Node(
            package="teach_pen",
            executable="ruben_camera_node",
            name="ruben_camera_node",
            output="screen",
            parameters=[{
                "camera_id": camera_id,
                "camera_param_file": camera_param_file,
            }],
            condition=IfCondition(start_camera),
        ),

        Node(
            package="teach_pen",
            executable="seam_perception_node",
            name="seam_perception_node",
            output="screen",
            parameters=[{
                "teaching_path_file": teaching_path_file,
                "output_path_file": detected_path_file,
                "path_orientation_file": photo_pose_file,
                "hand_eye_file": camera_hand_eye_file,
                "path_roi_radius": 0.1,
                "detected_path_z_offset": detected_path_z_offset,
            }],
            condition=IfCondition(start_seam_perception),
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
