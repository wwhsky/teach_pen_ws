import threading
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


def rotation_matrix_to_quaternion(rotation):
    trace = np.trace(rotation)

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    quaternion = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm > 0.0:
        quaternion /= norm
    return quaternion


class WorkpieceCalibrator(Node):
    def __init__(self):
        super().__init__("calibrate_workpiece")

        self.declare_parameter("reference_frame", "robot_base")
        self.declare_parameter("point_frame", "teaching_pen_tip")
        self.declare_parameter("workpiece_frame", "workpiece_frame")
        self.declare_parameter("output_file", "config/calibration/workpiece.yaml")

        self.reference_frame = self.get_parameter("reference_frame").value
        self.point_frame = self.get_parameter("point_frame").value
        self.workpiece_frame = self.get_parameter("workpiece_frame").value
        self.output_file = self.get_parameter("output_file").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = []

    def sample_once(self):
        T_reference_point = self.lookup_transform(
            self.reference_frame,
            self.point_frame,
        )
        if T_reference_point is None:
            self.get_logger().warn("sample skipped because TF lookup failed")
            return False

        self.samples.append({
            "index": len(self.samples) + 1,
            "role": self.sample_role(len(self.samples)),
            "point": T_reference_point,
        })

        xyz = T_reference_point["translation"]
        self.get_logger().info(
            "sample %d/%d (%s): %s <- %s xyz=[%.6f, %.6f, %.6f]"
            % (
                len(self.samples),
                3,
                self.samples[-1]["role"],
                self.reference_frame,
                self.point_frame,
                xyz[0],
                xyz[1],
                xyz[2],
            )
        )
        return True

    def sample_role(self, index):
        if index == 0:
            return "origin"
        if index == 1:
            return "x_axis"
        if index == 2:
            return "xy_plane"
        return "extra"

    def lookup_transform(self, parent_frame, child_frame):
        try:
            T_parent_child_msg = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            self.get_logger().warn(f"TF lookup failed: {error}")
            return None

        t = T_parent_child_msg.transform.translation
        q = T_parent_child_msg.transform.rotation
        stamp = T_parent_child_msg.header.stamp

        return {
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "parent_frame": T_parent_child_msg.header.frame_id,
            "child_frame": T_parent_child_msg.child_frame_id,
            "translation": [float(t.x), float(t.y), float(t.z)],
            "rotation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
        }

    def compute_calibration(self):
        if len(self.samples) < 3:
            self.get_logger().warn("need exactly 3 samples to compute workpiece frame")
            return None

        if len(self.samples) > 3:
            self.get_logger().warn("more than 3 samples collected; only the first 3 are used")

        origin = np.array(self.samples[0]["point"]["translation"], dtype=float)
        x_point = np.array(self.samples[1]["point"]["translation"], dtype=float)
        xy_point = np.array(self.samples[2]["point"]["translation"], dtype=float)

        x_axis = x_point - origin
        x_norm = np.linalg.norm(x_axis)
        if x_norm == 0.0:
            self.get_logger().error("origin point and x-axis point are identical")
            return None
        x_axis /= x_norm

        xy_vector = xy_point - origin
        z_axis = np.cross(x_axis, xy_vector)
        z_norm = np.linalg.norm(z_axis)
        if z_norm == 0.0:
            self.get_logger().error("three workpiece points are collinear")
            return None
        z_axis /= z_norm

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rotation = np.column_stack([x_axis, y_axis, z_axis])
        quaternion = rotation_matrix_to_quaternion(rotation)

        self.get_logger().info(
            "computed %s -> %s from 3 points, origin [%.6f, %.6f, %.6f]"
            % (
                self.reference_frame,
                self.workpiece_frame,
                origin[0],
                origin[1],
                origin[2],
            )
        )

        return {
            "type": "three_point_workpiece_frame",
            "description": (
                "Point 1 defines the workpiece origin, point 2 defines +X, "
                "and point 3 defines the XY plane. +Z follows the right-hand rule."
            ),
            "parent_frame": self.reference_frame,
            "child_frame": self.workpiece_frame,
            "translation": origin.tolist(),
            "rotation_xyzw": quaternion.tolist(),
            "rotation_matrix": rotation.tolist(),
            "point_frame": self.point_frame,
            "origin_point": origin.tolist(),
            "x_axis_point": x_point.tolist(),
            "xy_plane_point": xy_point.tolist(),
            "x_axis": x_axis.tolist(),
            "y_axis": y_axis.tolist(),
            "z_axis": z_axis.tolist(),
        }

    def save(self):
        data = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "frames": {
                "reference_frame": self.reference_frame,
                "point_frame": self.point_frame,
                "workpiece_frame": self.workpiece_frame,
            },
            "sample_count": len(self.samples),
            "samples": self.samples,
            "calibration_result": self.compute_calibration(),
        }

        with open(self.output_file, "w", encoding="utf-8") as file:
            yaml.safe_dump(data, file, sort_keys=False)

        self.get_logger().info(
            f"saved {len(self.samples)} samples to {self.output_file}"
        )


def main():
    rclpy.init()
    node = WorkpieceCalibrator()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("")
    print("Workpiece three-point calibrator")
    print(f"  Sample TF: {node.reference_frame} <- {node.point_frame}")
    print(f"  Result TF: {node.reference_frame} -> {node.workpiece_frame}")
    print(f"  Output:    {node.output_file}")
    print("")
    print("Point order:")
    print("  1. workpiece origin")
    print("  2. point on +X axis")
    print("  3. point on +XY plane")
    print("")
    print("Press Enter to sample, 'q' then Enter to save and quit.")
    print("")

    try:
        while rclpy.ok():
            command = input("> ").strip().lower()
            if command in ("q", "quit", "exit"):
                break
            if len(node.samples) >= 3:
                node.get_logger().warn("already collected 3 points; input q to save")
                continue
            node.sample_once()
    except KeyboardInterrupt:
        print("")
    finally:
        node.save()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
