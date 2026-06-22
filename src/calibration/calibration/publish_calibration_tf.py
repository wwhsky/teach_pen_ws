from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def normalize_quaternion(quaternion):
    norm = sum(value * value for value in quaternion) ** 0.5
    if norm == 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [float(value) / norm for value in quaternion]


class CalibrationTfPublisher(Node):
    def __init__(self):
        super().__init__("publish_calibration_tf")

        self.declare_parameter("files", [
            "config/calibration/teaching_pen_tip.yaml",
            "config/calibration/welding_torch_tip.yaml",
            "config/calibration/vr_to_robot.yaml",
            "config/calibration/workpiece.yaml",
        ])

        self.files = list(self.get_parameter("files").value)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.transforms = self.load_transforms()

        if self.transforms:
            self.broadcaster.sendTransform(self.transforms)
            for transform in self.transforms:
                self.get_logger().info(
                    "published static TF %s -> %s"
                    % (transform.header.frame_id, transform.child_frame_id)
                )
        else:
            self.get_logger().warn("no calibration TFs were loaded")

    def load_transforms(self):
        transforms = []
        for file_name in self.files:
            path = Path(file_name).expanduser()
            if not path.exists():
                self.get_logger().warn(f"calibration file not found: {path}")
                continue

            transform_data = self.load_transform_data(path)
            if transform_data is None:
                continue

            transforms.append(self.make_transform(transform_data, path))

        return transforms

    def load_transform_data(self, path):
        try:
            with path.open("r", encoding="utf-8") as file:
                data = yaml.safe_load(file)
        except yaml.YAMLError as error:
            self.get_logger().error(f"failed to parse {path}: {error}")
            return None

        if not isinstance(data, dict):
            self.get_logger().error(f"calibration file is not a map: {path}")
            return None

        transform_data = data.get("calibration_result", data)
        if transform_data is None:
            self.get_logger().error(f"calibration_result is empty: {path}")
            return None

        required_fields = ("parent_frame", "child_frame", "translation", "rotation_xyzw")
        missing_fields = [
            field for field in required_fields
            if field not in transform_data
        ]
        if missing_fields:
            self.get_logger().error(
                f"{path} missing required fields: {', '.join(missing_fields)}"
            )
            return None

        return transform_data

    def make_transform(self, transform_data, path):
        translation = transform_data["translation"]
        rotation = normalize_quaternion(transform_data["rotation_xyzw"])

        if len(translation) != 3:
            raise ValueError(f"{path} translation must contain 3 values")
        if len(rotation) != 4:
            raise ValueError(f"{path} rotation_xyzw must contain 4 values")

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(transform_data["parent_frame"])
        transform.child_frame_id = str(transform_data["child_frame"])
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform


def main():
    rclpy.init()
    node = CalibrationTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
