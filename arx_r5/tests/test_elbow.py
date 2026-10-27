"""回归：相邻肘部躯体不能漏检；新接近示例不能退回大幅旋转。"""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from arx_r5.common import load_profile
from arx_r5.planning.collision import SphereChecker
from arx_r5.planning.model import forward_kinematics
from arx_r5.trajectory import export_trajectory, make_trajectory

OLD_START = [-0.8, -3.15, -1.3, -0.42, 0.13, 0.06]


class ElbowCollisionTests(unittest.TestCase):
    def test_reverse_fold_is_detected_beyond_old_exclusions(self) -> None:
        checker = SphereChecker()
        gap, pair = checker.clearance(OLD_START)
        self.assertLess(gap, -0.01)
        self.assertEqual(set(pair), {"link2_elbow_body", "link3_elbow_body"})
        # 同一个状态在此前“相邻整对排除”策略下确实会漏检。
        checker.pairs = [(a, b) for a, b in checker.pairs if "elbow_body" not in a + b]
        legacy_gap, _ = checker.clearance(OLD_START)
        self.assertGreater(legacy_gap, 0.0)

    def test_reverse_fold_cannot_export_with_forged_pass(self) -> None:
        q = OLD_START.copy()
        data = make_trajectory(q, q, [q.copy() for _ in range(3)],
                               [[0.0] * 6 for _ in range(3)], [[0.0] * 6 for _ in range(3)], 0.025)
        data["self_collision_validation"] = {"passed": True}
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "must_not_export.json"
            with self.assertRaisesRegex(ValueError, "elbow_body"):
                export_trajectory(output, data)
            self.assertFalse(output.exists())

    def test_forward_approach_moves_tool_without_base_rotation(self) -> None:
        start = load_profile()["default_position"]
        goal = [0.0, 0.0, 0.45, -0.35, 0.13, 0.06]
        positions = np.linspace(np.array(start, dtype=np.float32), np.array(goal, dtype=np.float32),
                                21, dtype=np.float32)
        result = SphereChecker().validate(positions.tolist())
        self.assertTrue(result["passed"])
        poses = [forward_kinematics(q.tolist())["link6"] for q in positions]
        self.assertGreater(float(np.linalg.norm(poses[-1][:3, 3] - poses[0][:3, 3])), 0.15)
        errors = [Rotation.from_matrix(poses[0][:3, :3].T @ t[:3, :3]).magnitude() for t in poses]
        self.assertLess(max(errors), 2e-6)
        self.assertEqual(start[0], goal[0])


if __name__ == "__main__":
    unittest.main()
