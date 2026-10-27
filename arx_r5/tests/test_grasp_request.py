"""抓取请求的恒等映射、工具偏移与直线/定姿复核；不连接硬件。"""
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from arx_r5.common import load_profile
from arx_r5.planning.smoke_test import linear_axis, load_request, transform, validate_pose_path


class GraspRequestTests(unittest.TestCase):
    def setUp(self) -> None:
        identity = np.eye(4, dtype=np.float32).tolist()
        self.request = {
            "schema_version": 1, "data_kind": "grasp_plan_request",
            "joint_names": load_profile()["joint_names"],
            "start_joint_positions_rad": [0.0] * 6,
            "sdk_reference_T_ee_at_capture": identity,
            "task": {
                "reference_frame": "sdk_cartesian_reference",
                "controlled_frame": "sdk_controlled_ee", "quaternion_order": "wxyz",
                "units": {"translation": "m", "angle": "rad"},
                "targets": {name: {"sdk_reference_T_ee_target": identity}
                            for name in ("pregrasp", "grasp", "lift")},
                "stages": [{"action": "plan_move_and_confirm_arrival", "target": name}
                           for name in ("pregrasp", "grasp", "lift")],
            },
        }

    def read(self, request: dict) -> tuple[dict, list[float], list[tuple[str, np.ndarray]]]:
        """只读取临时测试文件，不修改用户请求。"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "request.json"
            path.write_text(json.dumps(request))
            return load_request(path)

    def test_request_uses_identity_without_mapping_or_goal_joints(self) -> None:
        self.request["start_joint_positions_rad"] = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        del self.request["sdk_reference_T_ee_at_capture"]
        _, start, targets = self.read(self.request)
        self.assertEqual(start, self.request["start_joint_positions_rad"])
        self.assertEqual([name for name, _ in targets], ["pregrasp", "grasp", "lift"])
        for _, actual in targets:
            np.testing.assert_array_equal(actual, np.eye(4, dtype=np.float32))

    def test_tool_offset_is_preserved_and_not_applied_again(self) -> None:
        tool = np.eye(4, dtype=np.float32)
        tool[:3, 3] = [0.15, 0.0, 1.0]
        self.request["task"]["input_frame_transforms"] = {"sdk_ee_T_grasp_geometry": tool.tolist()}
        for i, name in enumerate(("pregrasp", "grasp", "lift")):
            target = np.eye(4, dtype=np.float32)
            target[:3, 3] = [0.3 + 0.1 * i, 0.1, -0.8]
            self.request["task"]["targets"][name]["sdk_reference_T_ee_target"] = target.tolist()
        request, _, targets = self.read(self.request)
        self.assertEqual(request["task"]["input_frame_transforms"]["sdk_ee_T_grasp_geometry"], tool.tolist())
        for name, actual in targets:
            expected = np.asarray(self.request["task"]["targets"][name]["sdk_reference_T_ee_target"],
                                  dtype=np.float32)
            np.testing.assert_array_equal(actual, expected)

    def test_capture_pose_is_not_used_to_redefine_the_confirmed_mapping(self) -> None:
        self.request["sdk_reference_T_ee_at_capture"][0][3] = 10.0
        _, start, _ = self.read(self.request)
        self.assertEqual(start, [0.0] * 6)

    def test_wrong_units_and_order_are_rejected(self) -> None:
        for field, value in [("joint_names", ["joint1"] * 6),
                             ("start_joint_positions_rad", [float("nan")] * 6)]:
            with self.subTest(field=field):
                request = copy.deepcopy(self.request)
                request[field] = value
                with self.assertRaises(ValueError):
                    self.read(request)
        self.request["task"]["quaternion_order"] = "xyzw"
        with self.assertRaisesRegex(ValueError, "frame or units"):
            self.read(self.request)

    def test_invalid_rotation_is_not_silently_normalized(self) -> None:
        matrix = np.eye(4, dtype=np.float32)
        matrix[0, 0] = -1
        with self.assertRaisesRegex(ValueError, "invalid rigid transform"):
            transform(matrix.tolist(), "test")


class LinearPathTests(unittest.TestCase):
    def setUp(self) -> None:
        self.start = np.eye(4, dtype=np.float32)
        self.goal = self.start.copy()
        self.goal[0, 3] = 0.1
        self.poses = np.repeat(self.start[None], 3, axis=0)
        self.poses[:, 0, 3] = [0.0, 0.05, 0.1]

    def test_straight_constant_orientation_path_passes(self) -> None:
        result = validate_pose_path(self.poses, self.start, self.goal, linear=True)
        self.assertLess(result["line_error_m"], 1e-6)
        self.assertEqual(result["max_orientation_error_rad"], 0.0)

    def test_bowed_path_rejects_even_with_correct_endpoint(self) -> None:
        self.poses[1, 1, 3] = 0.01
        with self.assertRaisesRegex(ValueError, "Straight/constant-orientation"):
            validate_pose_path(self.poses, self.start, self.goal, linear=True)

    def test_intermediate_rotation_rejects_even_with_correct_endpoint(self) -> None:
        self.poses[1, :3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        with self.assertRaisesRegex(ValueError, "Straight/constant-orientation"):
            validate_pose_path(self.poses, self.start, self.goal, linear=True)

    def test_axis_uses_goal_tool_for_grasp_and_base_for_lift(self) -> None:
        c = 2 ** -0.5
        self.start[:3, :3] = [[c, 0, c], [0, 1, 0], [-c, 0, c]]
        self.goal = self.start.copy()
        self.goal[:3, 3] = self.start[:3, 0] * 0.2
        self.assertEqual(linear_axis(self.start, self.goal), ("x", True))
        self.goal[:3, 3] = [0, 0, 0.1]
        self.assertEqual(linear_axis(self.start, self.goal), ("z", False))

    def test_changing_orientation_is_not_a_translation_stage(self) -> None:
        self.goal[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        with self.assertRaisesRegex(ValueError, "same start and goal orientation"):
            linear_axis(self.start, self.goal)


if __name__ == "__main__":
    unittest.main()
