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

        self.files = [Path(file_name).expanduser() for file_name in self.get_parameter("files").value]
        self.broadcaster = StaticTransformBroadcaster(self)
        self.file_mtimes = {}
        self.transforms = self.load_transforms()
        self.update_file_mtimes()

        if self.transforms:
            self.publish_transforms()
            self.timer = self.create_timer(1.0, self.check_and_publish)
            for T_parent_child_msg in self.transforms:
                self.get_logger().info(
                    "published static TF %s -> %s"
                    % (T_parent_child_msg.header.frame_id, T_parent_child_msg.child_frame_id)
                )
        else:
            self.get_logger().warn("no calibration TFs were loaded")

    def publish_transforms(self):
        stamp = self.get_clock().now().to_msg()
        for T_parent_child_msg in self.transforms:
            T_parent_child_msg.header.stamp = stamp
        self.broadcaster.sendTransform(self.transforms)

    def check_and_publish(self):
        if self.files_changed():
            transforms = self.load_transforms()
            if transforms:
                self.transforms = transforms
                self.update_file_mtimes()
                self.get_logger().info("calibration yaml changed; reloaded TFs")
            else:
                self.get_logger().warn("calibration yaml changed, but no valid TFs were loaded")

        if self.transforms:
            self.publish_transforms()

    def files_changed(self):
        for path in self.files:
            try:
                mtime = path.stat().st_mtime_ns
            except OSError:
                mtime = None
            if self.file_mtimes.get(path) != mtime:
                return True
        return False

    def update_file_mtimes(self):
        self.file_mtimes = {}
        for path in self.files:
            try:
                self.file_mtimes[path] = path.stat().st_mtime_ns
            except OSError:
                self.file_mtimes[path] = None

    def load_transforms(self):
        T_parent_child_msgs = []
        for path in self.files:
            if not path.exists():
                self.get_logger().warn(f"calibration file not found: {path}")
                continue

            transform_data = self.load_transform_data(path)
            if transform_data is None:
                continue

            T_parent_child_msgs.append(self.make_transform(transform_data, path))

        return T_parent_child_msgs

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

        T_parent_child_msg = TransformStamped()
        T_parent_child_msg.header.stamp = self.get_clock().now().to_msg()
        T_parent_child_msg.header.frame_id = str(transform_data["parent_frame"])
        T_parent_child_msg.child_frame_id = str(transform_data["child_frame"])
        T_parent_child_msg.transform.translation.x = float(translation[0])
        T_parent_child_msg.transform.translation.y = float(translation[1])
        T_parent_child_msg.transform.translation.z = float(translation[2])
        T_parent_child_msg.transform.rotation.x = rotation[0]
        T_parent_child_msg.transform.rotation.y = rotation[1]
        T_parent_child_msg.transform.rotation.z = rotation[2]
        T_parent_child_msg.transform.rotation.w = rotation[3]
        return T_parent_child_msg


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
