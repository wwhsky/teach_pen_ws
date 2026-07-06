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


# 四元数转旋转矩阵
def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        return np.eye(3)

    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


class ToolCalibrator(Node):
    def __init__(self):
        super().__init__("calibrate_tool")

        self.declare_parameter("reference_frame", "robot_base")
        self.declare_parameter("parent_frame", "robot_flange")
        self.declare_parameter("tip_frame", "welding_torch_tip")
        self.declare_parameter("output_file", "tool_calibration.yaml")
        self.declare_parameter("min_samples", 4)

        self.reference_frame = self.get_parameter("reference_frame").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.tip_frame = self.get_parameter("tip_frame").value
        self.output_file = self.get_parameter("output_file").value
        self.min_samples = int(self.get_parameter("min_samples").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = []

    def sample_once(self):
        # 查找机器人法兰位姿或者 Tracker 位姿
        T_reference_parent = self.lookup_transform(
            self.reference_frame,
            self.parent_frame,
        )
        if T_reference_parent is None:
            self.get_logger().warn("sample skipped because TF lookup failed")
            return False

        self.samples.append({
            "index": len(self.samples) + 1,
            "transform": T_reference_parent,
        })

        xyz = T_reference_parent["translation"]
        self.get_logger().info(
            "sample %d: %s <- %s xyz=[%.6f, %.6f, %.6f]"
            % (
                len(self.samples),
                self.reference_frame,
                self.parent_frame,
                xyz[0],
                xyz[1],
                xyz[2],
            )
        )
        return True

    def lookup_transform(self, parent_frame, child_frame):
        # 查找并保存robot_flange在robot_base下的旋转和平移
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
        if len(self.samples) < self.min_samples:
            self.get_logger().warn(
                f"need at least {self.min_samples} samples to compute tool calibration"
            )
            return None

        # 根据采样点的rotation和translation构建matrix和vector,用于最小二乘法求解
        # 优化问题为R_i x + t_i = p，其中t_i为采样点的translation，p为空间固定点，x为工具tip在父坐标系下的偏移量
        # R_i x + t_i = p
        # R_i x - p = -t_i
        # [R_i  -I] [x] = -t_i
        #           [p]
        # [R_i  -I] 组成matrix_rows,[-t_i] 组成vector_rows
        # 如果有N个样本，则matrix尺寸为 3Nx6, vector尺寸为 3Nx1
        matrix_rows = []
        vector_rows = []
        rotations = []
        translations = []

        for sample in self.samples:
            T_reference_parent = sample["transform"]
            R_reference_parent = quaternion_to_rotation_matrix(
                np.array(T_reference_parent["rotation_xyzw"], dtype=float)
            )
            t_reference_parent = np.array(T_reference_parent["translation"], dtype=float)

            matrix_rows.append(np.hstack([R_reference_parent, -np.eye(3)]))
            vector_rows.append(-t_reference_parent)
            rotations.append(R_reference_parent)
            translations.append(t_reference_parent)

        matrix = np.vstack(matrix_rows)
        vector = np.hstack(vector_rows)

        # 调用numpy的最小二乘法求解器求解线性方程组，其中sigular_values用于判断方程条件好不好
        # 如果有奇异值接近0，说明标定数据退化，比如姿态变化太少
        solution, residuals, rank, singular_values = np.linalg.lstsq(
            matrix,
            vector,
            rcond=None,
        )

        # 整理并保存标定解
        tip_offset = solution[0:3]
        pivot_point = solution[3:6]
        estimated_tip_points = np.array([
            rotation @ tip_offset + translation
            for rotation, translation in zip(rotations, translations)
        ])
        errors = np.linalg.norm(estimated_tip_points - pivot_point, axis=1)

        self.get_logger().info(
            "computed %s -> %s from %d samples, tip offset [%.6f, %.6f, %.6f], "
            "mean error %.6f m, max error %.6f m"
            % (
                self.parent_frame,
                self.tip_frame,
                len(self.samples),
                tip_offset[0],
                tip_offset[1],
                tip_offset[2],
                float(errors.mean()),
                float(errors.max()),
            )
        )

        return {
            "type": "pivot_calibration_translation_only",
            "description": (
                "Computed by keeping the tool tip fixed at one physical pivot point "
                "and moving the parent frame through different orientations. "
                "Rotation is left as identity and can be edited manually."
            ),
            "parent_frame": self.parent_frame,
            "child_frame": self.tip_frame,
            "translation": tip_offset.tolist(),
            "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "reference_frame": self.reference_frame,
            "pivot_point_in_reference_frame": pivot_point.tolist(),
            "sample_count": len(self.samples),
            "rank": int(rank),
            "residuals": residuals.tolist(),
            "singular_values": singular_values.tolist(),
            "mean_error": float(errors.mean()),
            "max_error": float(errors.max()),
            "per_sample_errors": errors.tolist(),
        }

    def save(self):
        data = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "frames": {
                "reference_frame": self.reference_frame,
                "parent_frame": self.parent_frame,
                "tip_frame": self.tip_frame,
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
    node = ToolCalibrator()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("")
    print("Tool pivot calibrator")
    print(f"  Sample TF: {node.reference_frame} <- {node.parent_frame}")
    print(f"  Result TF: {node.parent_frame} -> {node.tip_frame}")
    print(f"  Output:    {node.output_file}")
    print("")
    print("Keep the tool tip fixed on one physical point.")
    print("Press Enter to sample, 'q' then Enter to save and quit.")
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
