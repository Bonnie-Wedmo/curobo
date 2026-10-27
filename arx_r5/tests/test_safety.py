"""离线协议与控制拒绝路径；测试状态不代表真实机械臂。"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from arx_r5.common import ROOT, load_profile, write_json
from arx_r5.control.mock import MockController
from arx_r5.control.run import execute_mock
from arx_r5.control.sdk import OfficialR5Adapter, decode_sdk_feedback
from arx_r5.trajectory import export_trajectory, make_trajectory, validate_numeric


def fixture() -> dict:
    """仅单元测试使用的静止轨迹，绝不标记为 GPU 规划实测输出。"""
    q = load_profile()["default_position"]
    data = make_trajectory(q.copy(), q.copy(), [q.copy() for _ in range(5)],
                           [[0.0] * 6 for _ in range(5)], [[0.0] * 6 for _ in range(5)], 0.025)
    data["test_fixture"] = "synthetic_stationary_not_planner_output"
    return data


class TrajectorySafetyTests(unittest.TestCase):
    """协议字段、模型哈希和数值检查必须拒绝伪造/损坏的文件。"""

    def test_valid_numeric(self) -> None:
        self.assertTrue(validate_numeric(fixture())["passed"])

    def test_protocol_mutations(self) -> None:
        for key, value in [("joint_names", list(reversed(load_profile()["joint_names"]))),
                           ("units", {"position": "degree"}), ("planning_status", "failed"),
                           ("hardware_eligible", True), ("provenance", {}),
                           ("time_from_start", [0, 0, 0, 0, 0]),
                           ("interpolation_dt", 0.05),
                           ("start_state", [0.0]), ("locked_gripper", {}),
                           ("start_state_source", "hardware")]:
            with self.subTest(key=key):
                data = fixture()
                data[key] = value
                with self.assertRaises(ValueError):
                    validate_numeric(data)

    def test_nonfinite_and_malformed(self) -> None:
        for value in [float("nan"), float("inf"), True, "0.0"]:
            data = fixture()
            data["joint_positions"][2][0] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_numeric(data)

    def test_limits_endpoints_and_hidden_jump(self) -> None:
        for field, index, value in [("joint_positions", 0, 0.2),
                                    ("joint_positions", 4, 0.2),
                                    ("joint_positions", 2, 9.0),
                                    ("joint_positions", 2, 0.1),
                                    ("joint_velocities", 2, 0.5),
                                    ("joint_accelerations", 2, 0.9)]:
            data = fixture()
            data[field][index][0] = value
            with self.subTest(field=field, index=index), self.assertRaises(ValueError):
                validate_numeric(data)

    def test_false_collision_claim_cannot_export(self) -> None:
        data = fixture()
        q = [0.0, -1.0, 0.6, 0.0, 0.0, 0.0]
        data["start_state"] = q.copy()
        data["goal_state"] = q.copy()
        data["joint_positions"] = [q.copy() for _ in range(5)]
        data["self_collision_validation"] = {"passed": True}
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "must_not_exist.json"
            with self.assertRaises(ValueError):
                export_trajectory(output, data)
            self.assertFalse(output.exists())

    def test_reader_without_site_packages(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "fixture.json"
            write_json(path, fixture())
            result = subprocess.run([sys.executable, "-S", "-m", "arx_r5.trajectory", str(path)],
                                    cwd=ROOT.parent, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('"hardware_eligible": false', result.stdout)


class ControlSafetyTests(unittest.TestCase):
    """执行完成、起点不符、通信失败及安全入口的测试。"""

    def test_mock_completion(self) -> None:
        data = fixture()
        controller = MockController(data["start_state"])
        with tempfile.TemporaryDirectory() as temp:
            summary = execute_mock(data, controller, Path(temp) / "run.jsonl")
        self.assertEqual(summary["status"], "completed")
        self.assertEqual(summary["rms_error_rad"], [0.0] * 6)
        self.assertTrue(controller.stopped)

    def test_mock_faults_stop_sending(self) -> None:
        data = fixture()
        bad_start = data["start_state"].copy()
        bad_start[0] += 0.1
        cases = [(MockController(bad_start), 0),
                 (MockController(data["start_state"], fail_at=2), 2),
                 (MockController(data["start_state"], stale=True), 0),
                 (MockController(data["start_state"], offset=0.02), 1),
                 (MockController(data["start_state"], offset=0.005), 5)]
        for controller, command_count in cases:
            with self.subTest(count=command_count), tempfile.TemporaryDirectory() as temp:
                log = Path(temp) / "run.jsonl"
                with self.assertRaises((RuntimeError, ConnectionError)):
                    execute_mock(data, controller, log)
                self.assertEqual(controller.commands, command_count)
                self.assertTrue(controller.stopped)
                summary = json.loads(log.with_suffix(".summary.json").read_text())
                self.assertEqual(summary["status"], "aborted")

    def test_keyboard_interrupt_stops_mock(self) -> None:
        class InterruptedMock(MockController):
            def send(self, position: list[float], velocity: list[float], now: float) -> None:
                raise KeyboardInterrupt("Injected operator interruption")

        controller = InterruptedMock(fixture()["start_state"])
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(KeyboardInterrupt):
                execute_mock(fixture(), controller, Path(temp) / "interrupted.jsonl")
        self.assertTrue(controller.stopped)
        self.assertEqual(controller.commands, 0)

    def test_sdk_feedback_contract(self) -> None:
        feedback = decode_sdk_feedback([0.1] * 6 + [2.0], [0.0] * 7)
        self.assertEqual(feedback["gripper_position_raw"], 2.0)
        self.assertEqual(len(feedback["arm_position_raw"]), 6)
        self.assertFalse(feedback["joint_mapping_verified"])
        with self.assertRaises(ValueError):
            decode_sdk_feedback([0.0] * 6, [0.0] * 6)

    def test_hardware_rejected_even_with_forged_flags(self) -> None:
        data = fixture()
        data["hardware_eligible"] = True
        adapter = OfficialR5Adapter()
        with self.assertRaisesRegex(RuntimeError, "HARDWARE BLOCKED"):
            adapter.execute(data, operator_authorized=True)
        self.assertNotIn("arx_r5_python", sys.modules)


if __name__ == "__main__":
    unittest.main()
