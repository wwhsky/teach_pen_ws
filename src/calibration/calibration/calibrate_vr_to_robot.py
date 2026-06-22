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


class VrToRobotSampleCollector(Node):
    def __init__(self):
        super().__init__("calibrate_vr_to_robot")

        self.declare_parameter("tracker_parent_frame", "steamvr_base")
        self.declare_parameter("tracker_tip_frame", "teaching_pen_tip")
        self.declare_parameter("robot_parent_frame", "robot_base")
        self.declare_parameter("robot_tip_frame", "welding_torch_tip")
        self.declare_parameter("output_file", "vr_to_robot_samples.yaml")

        self.tracker_parent_frame = self.get_parameter("tracker_parent_frame").value
        self.tracker_tip_frame = self.get_parameter("tracker_tip_frame").value
        self.robot_parent_frame = self.get_parameter("robot_parent_frame").value
        self.robot_tip_frame = self.get_parameter("robot_tip_frame").value
        self.output_file = self.get_parameter("output_file").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = []

    def sample_once(self):
        tracker_tip_transform = self.lookup_transform(
            self.tracker_parent_frame,
            self.tracker_tip_frame,
            "tracker tip",
        )
        robot_tip_transform = self.lookup_transform(
            self.robot_parent_frame,
            self.robot_tip_frame,
            "robot tip",
        )

        if tracker_tip_transform is None or robot_tip_transform is None:
            self.get_logger().warn("sample skipped because at least one TF lookup failed")
            return False

        self.samples.append({
            "index": len(self.samples) + 1,
            "tracker_tip": tracker_tip_transform,
            "robot_tip": robot_tip_transform,
        })

        tracker_xyz = tracker_tip_transform["translation"]
        robot_xyz = robot_tip_transform["translation"]
        self.get_logger().info(
            "sample %d: teaching_pen_tip xyz=[%.6f, %.6f, %.6f], "
            "welding_torch_tip xyz=[%.6f, %.6f, %.6f]"
            % (
                len(self.samples),
                tracker_xyz[0],
                tracker_xyz[1],
                tracker_xyz[2],
                robot_xyz[0],
                robot_xyz[1],
                robot_xyz[2],
            )
        )
        return True

    def lookup_transform(self, parent_frame, child_frame, label):
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            self.get_logger().warn(f"{label} TF lookup failed: {error}")
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        stamp = transform.header.stamp

        return {
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "parent_frame": transform.header.frame_id,
            "child_frame": transform.child_frame_id,
            "translation": [float(t.x), float(t.y), float(t.z)],
            "rotation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
        }

    def compute_calibration(self):
        if len(self.samples) < 3:
            self.get_logger().warn("need at least 3 samples to compute calibration")
            return None

        tracker_points = np.array(
            [sample["tracker_tip"]["translation"] for sample in self.samples],
            dtype=float,
        )
        robot_points = np.array(
            [sample["robot_tip"]["translation"] for sample in self.samples],
            dtype=float,
        )

        tracker_centroid = tracker_points.mean(axis=0)
        robot_centroid = robot_points.mean(axis=0)
        tracker_centered = tracker_points - tracker_centroid
        robot_centered = robot_points - robot_centroid

        covariance = tracker_centered.T @ robot_centered
        u, singular_values, vt = np.linalg.svd(covariance)

        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T

        translation = robot_centroid - rotation @ tracker_centroid
        transformed_tracker_points = (rotation @ tracker_points.T).T + translation
        errors = np.linalg.norm(transformed_tracker_points - robot_points, axis=1)
        quaternion = rotation_matrix_to_quaternion(rotation)

        self.get_logger().info(
            "computed %s -> %s from %d samples, mean error %.6f m, max error %.6f m"
            % (
                self.robot_parent_frame,
                self.tracker_parent_frame,
                len(self.samples),
                float(errors.mean()),
                float(errors.max()),
            )
        )

        return {
            "type": "rigid_transform_svd",
            "description": (
                "Computed from teaching_pen_tip positions in steamvr_base and "
                "welding_torch_tip positions in robot_base."
            ),
            "parent_frame": self.robot_parent_frame,
            "child_frame": self.tracker_parent_frame,
            "translation": translation.tolist(),
            "rotation_xyzw": quaternion.tolist(),
            "rotation_matrix": rotation.tolist(),
            "sample_count": len(self.samples),
            "singular_values": singular_values.tolist(),
            "mean_error": float(errors.mean()),
            "max_error": float(errors.max()),
            "per_sample_errors": errors.tolist(),
        }

    def save(self):
        data = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "frames": {
                "tracker_parent_frame": self.tracker_parent_frame,
                "tracker_tip_frame": self.tracker_tip_frame,
                "robot_parent_frame": self.robot_parent_frame,
                "robot_tip_frame": self.robot_tip_frame,
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
    node = VrToRobotSampleCollector()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("")
    print("VR-to-robot sample collector")
    print(f"  Tracker tip TF: {node.tracker_parent_frame} <- {node.tracker_tip_frame}")
    print(f"  Robot tip TF:   {node.robot_parent_frame} <- {node.robot_tip_frame}")
    print(f"  Output:     {node.output_file}")
    print("")
    print("Press Enter to sample both TFs, 'q' then Enter to save and quit.")
    print("")

    try:
        while rclpy.ok():
            command = input("> ").strip().lower()
            if command in ("q", "quit", "exit"):
                break
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
