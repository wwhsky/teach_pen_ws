import threading
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


# Transform naming convention: T_A_B means the pose of frame B in frame A.
# R_A_B and t_A_B follow the same direction.
def rotation_matrix_to_quaternion(rotation):
    return Rotation.from_matrix(rotation).as_quat().tolist()


def quaternion_to_rotation_matrix(quaternion):
    q = np.array(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.eye(3)
    return Rotation.from_quat(q / norm).as_matrix()



def rigid_transform_svd(points_in_source, points_in_target):
    source_centroid = points_in_source.mean(axis=0)
    target_centroid = points_in_target.mean(axis=0)
    source_centered = points_in_source - source_centroid
    target_centered = points_in_target - target_centroid

    covariance = source_centered.T @ target_centered
    u, singular_values, vt = np.linalg.svd(covariance)
    R_target_source = vt.T @ u.T
    if np.linalg.det(R_target_source) < 0.0:
        vt[-1, :] *= -1.0
        R_target_source = vt.T @ u.T

    t_target_source = target_centroid - R_target_source @ source_centroid
    return R_target_source, t_target_source, singular_values


class ToolVrJointCalibrator(Node):
    def __init__(self):
        super().__init__("calibrate_tool_vr_joint")

        self.declare_parameter("robot_parent_frame", "robot_base")
        self.declare_parameter("robot_flange_frame", "robot_flange")
        self.declare_parameter("robot_tip_frame", "welding_torch_tip")
        self.declare_parameter("tracker_parent_frame", "steamvr_base")
        self.declare_parameter("tracker_tip_frame", "teaching_pen_tip")
        self.declare_parameter("input_file", "")
        self.declare_parameter("output_file", "tool_vr_joint_calibration.yaml")
        self.declare_parameter("min_samples", 6)

        self.robot_parent_frame = self.get_parameter("robot_parent_frame").value
        self.robot_flange_frame = self.get_parameter("robot_flange_frame").value
        self.robot_tip_frame = self.get_parameter("robot_tip_frame").value
        self.tracker_parent_frame = self.get_parameter("tracker_parent_frame").value
        self.tracker_tip_frame = self.get_parameter("tracker_tip_frame").value
        self.input_file = self.get_parameter("input_file").value
        self.output_file = self.get_parameter("output_file").value
        self.min_samples = int(self.get_parameter("min_samples").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = []

    def lookup_transform(self, parent_frame, child_frame, label):
        try:
            T_parent_child_msg = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            self.get_logger().warn(f"{label} TF lookup failed: {error}")
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

    def sample_once(self):
        T_robot_parent_flange = self.lookup_transform(
            self.robot_parent_frame,
            self.robot_flange_frame,
            "robot flange",
        )
        T_tracker_parent_tip = self.lookup_transform(
            self.tracker_parent_frame,
            self.tracker_tip_frame,
            "tracker tip",
        )

        if T_robot_parent_flange is None or T_tracker_parent_tip is None:
            self.get_logger().warn("sample skipped because at least one TF lookup failed")
            return False

        self.samples.append({
            "index": len(self.samples) + 1,
            "robot_flange": T_robot_parent_flange,
            "tracker_tip": T_tracker_parent_tip,
        })

        robot_xyz = T_robot_parent_flange["translation"]
        tracker_xyz = T_tracker_parent_tip["translation"]
        self.get_logger().info(
            "sample %d: %s xyz=[%.6f, %.6f, %.6f], %s xyz=[%.6f, %.6f, %.6f]"
            % (
                len(self.samples),
                self.robot_flange_frame,
                robot_xyz[0],
                robot_xyz[1],
                robot_xyz[2],
                self.tracker_tip_frame,
                tracker_xyz[0],
                tracker_xyz[1],
                tracker_xyz[2],
            )
        )
        return True

    def load_samples_from_file(self, input_file):
        with open(input_file, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

        samples = data.get("samples", []) if data else []
        if not samples:
            self.get_logger().error(f"no samples found in {input_file}")
            return False

        self.samples = samples
        self.get_logger().info(f"loaded {len(self.samples)} samples from {input_file}")
        return True

    def compute_initial_guess(self, flange_rotations, flange_translations, tracker_points):
        tool_offset = np.zeros(3, dtype=float)
        robot_tip_points = np.array([
            rotation @ tool_offset + translation
            for rotation, translation in zip(flange_rotations, flange_translations)
        ])
        R_robot_vr, t_robot_vr, _ = rigid_transform_svd(
            tracker_points,
            robot_tip_points,
        )
        return np.hstack([
            tool_offset,
            Rotation.from_matrix(R_robot_vr).as_rotvec(),
            t_robot_vr,
        ])

    def compute_calibration(self):
        if len(self.samples) < self.min_samples:
            self.get_logger().warn(
                f"need at least {self.min_samples} samples to compute joint calibration"
            )
            return None

        flange_rotations = []
        flange_translations = []
        tracker_points = []
        for sample in self.samples:
            robot_flange = sample["robot_flange"]
            flange_rotations.append(quaternion_to_rotation_matrix(robot_flange["rotation_xyzw"]))
            flange_translations.append(np.array(robot_flange["translation"], dtype=float))
            tracker_points.append(np.array(sample["tracker_tip"]["translation"], dtype=float))

        tracker_points = np.array(tracker_points, dtype=float)
        initial = self.compute_initial_guess(
            flange_rotations,
            flange_translations,
            tracker_points,
        )

        def residual(parameters):
            tool_offset = parameters[0:3]
            R_robot_vr = Rotation.from_rotvec(parameters[3:6]).as_matrix()
            t_robot_vr = parameters[6:9]

            rows = []
            for flange_rotation, flange_translation, tracker_point in zip(
                flange_rotations,
                flange_translations,
                tracker_points,
            ):
                robot_tip = flange_rotation @ tool_offset + flange_translation
                p_robot_tip_from_vr = R_robot_vr @ tracker_point + t_robot_vr
                rows.append(robot_tip - p_robot_tip_from_vr)
            return np.hstack(rows)

        result = least_squares(
            residual,
            initial,
            method="trf",
            loss="soft_l1",
            max_nfev=2000,
        )

        tool_offset = result.x[0:3]
        R_robot_vr = Rotation.from_rotvec(result.x[3:6]).as_matrix()
        t_robot_vr = result.x[6:9]

        robot_tip_points = np.array([
            rotation @ tool_offset + translation
            for rotation, translation in zip(flange_rotations, flange_translations)
        ])
        p_robot_tip_from_vr = (R_robot_vr @ tracker_points.T).T + t_robot_vr
        errors = np.linalg.norm(robot_tip_points - p_robot_tip_from_vr, axis=1)

        self.get_logger().info(
            "computed joint calibration from %d samples, tool offset=[%.6f, %.6f, %.6f] m, "
            "mean error %.6f m, max error %.6f m"
            % (
                len(self.samples),
                tool_offset[0],
                tool_offset[1],
                tool_offset[2],
                float(errors.mean()),
                float(errors.max()),
            )
        )

        return {
            "type": "joint_tool_translation_and_vr_to_robot_position_least_squares",
            "description": (
                "Jointly estimates robot_flange -> robot_tip translation and "
                "robot_base -> steamvr_base transform from coincident tip samples. "
                "Tool rotation is not observable from point-contact samples and is left as identity."
            ),
            "tool_transform": {
                "parent_frame": self.robot_flange_frame,
                "child_frame": self.robot_tip_frame,
                "translation": tool_offset.tolist(),
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "vr_to_robot_transform": {
                "parent_frame": self.robot_parent_frame,
                "child_frame": self.tracker_parent_frame,
                "translation": t_robot_vr.tolist(),
                "rotation_xyzw": rotation_matrix_to_quaternion(R_robot_vr),
                "rotation_matrix": R_robot_vr.tolist(),
            },
            "sample_count": len(self.samples),
            "success": bool(result.success),
            "optimizer_message": result.message,
            "cost": float(result.cost),
            "mean_error": float(errors.mean()),
            "max_error": float(errors.max()),
            "per_sample_errors": errors.tolist(),
        }

    def save(self):
        data = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "frames": {
                "robot_parent_frame": self.robot_parent_frame,
                "robot_flange_frame": self.robot_flange_frame,
                "robot_tip_frame": self.robot_tip_frame,
                "tracker_parent_frame": self.tracker_parent_frame,
                "tracker_tip_frame": self.tracker_tip_frame,
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
    node = ToolVrJointCalibrator()

    if node.input_file:
        try:
            if node.load_samples_from_file(node.input_file):
                node.save()
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("")
    print("Tool + VR joint calibrator")
    print(f"  Robot flange TF: {node.robot_parent_frame} <- {node.robot_flange_frame}")
    print(f"  Tracker tip TF:  {node.tracker_parent_frame} <- {node.tracker_tip_frame}")
    print(f"  Result tool TF:  {node.robot_flange_frame} -> {node.robot_tip_frame}")
    print(f"  Result VR TF:    {node.robot_parent_frame} -> {node.tracker_parent_frame}")
    print(f"  Output:          {node.output_file}")
    print("")
    print("Put the robot tool tip and VR tracked tip on the same physical point.")
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
