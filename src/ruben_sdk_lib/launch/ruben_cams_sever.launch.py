from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 相机A(拼装)
        Node(
            package='ruben_sdk_lib',
            executable='ruben_sdk_server_node',
            name='camera_A',
            parameters=[{
                "camera_id": "M3GM620B014",
                "camera_paras": "CameraSettingcollaborative.json",
                "service_name": "/camera_A/get_data",
                "rgb_topic": "/camera_A/rgb",
                "pc_topic": "/camera_A/points",
                "frame_id": "camera_A_link"
            }]
        ),

        # 相机B(焊接)
        Node(
            package='ruben_sdk_lib',
            executable='ruben_sdk_server_node',
            name='camera_B',
            parameters=[{
                "camera_id": "M3GM620B009",
                "camera_paras": "CameraSettingweld.json",
                # "camera_paras": "CameraSettingcollaborative.json",
                "service_name": "/camera_B/get_data",
                "rgb_topic": "/camera_B/rgb",
                "pc_topic": "/camera_B/points",
                "frame_id": "camera_B_link"
            }]
        ),
    ])