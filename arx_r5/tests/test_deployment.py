"""跨机状态与部署预检的拒绝路径，不连接硬件。"""
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from unittest.mock import patch
from datetime import datetime, timedelta, timezone
from pathlib import Path

from arx_r5.common import ROOT, content_digest, digest, load_profile, provenance, write_json
from arx_r5.control.state import bind_start_snapshot, make_mock_snapshot, load_snapshot, validate_snapshot, import_recording
from arx_r5.control.preflight import preflight
from arx_r5.control.mock import Feedback, MockController, VirtualClock
from arx_r5.control.run import execute_mock
from arx_r5.control.sdk import OfficialR5Adapter
from arx_r5.trajectory import make_trajectory, validate_numeric


def trajectory() -> dict:
    """只为协议测试构造静止数据，不称为规划器输出。"""
    q = load_profile()["default_position"]
    return make_trajectory(q.copy(), q.copy(), [q.copy() for _ in range(3)],
                           [[0.0] * 6 for _ in range(3)], [[0.0] * 6 for _ in range(3)], 0.025)


class StartPreparationTests(unittest.TestCase):
    def test_uncommissioned_start_move_rejects_before_sdk_construction(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "HARDWARE BLOCKED before SDK construction"):
            OfficialR5Adapter().move_to_start(load_profile()["default_position"])

    def test_initialization_alone_never_means_arrival(self) -> None:
        with patch.object(OfficialR5Adapter, "initialize", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "Move-to-START is not implemented"):
                OfficialR5Adapter().move_to_start(load_profile()["default_position"])

    def test_invalid_target_rejects_before_initialization(self) -> None:
        with patch.object(OfficialR5Adapter, "initialize") as initialize:
            for target in ([0.0] * 5, [float("nan")] * 6):
                with self.subTest(target=target), self.assertRaises(ValueError):
                    OfficialR5Adapter().move_to_start(target)
            initialize.assert_not_called()


class SnapshotTests(unittest.TestCase):
    def test_mock_roundtrip_and_source(self) -> None:
        snapshot = make_mock_snapshot(load_profile()["default_position"])
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "state.json"
            write_json(path, snapshot)
            self.assertEqual(load_snapshot(path)["source"], "mock")
        self.assertFalse(snapshot["hardware_eligible"])

    def test_reject_bad_snapshots(self) -> None:
        for field, value in [("position", [0.0] * 5), ("velocity", [float("nan")] * 6),
                             ("joint_names", ["joint1"] * 6), ("hardware_eligible", True),
                             ("captured_at_utc", "2026-01-01"), ("source", "live_verified")]:
            snapshot = make_mock_snapshot(load_profile()["default_position"])
            snapshot[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_snapshot(snapshot)

    def test_bound_snapshot_cannot_disagree_with_start(self) -> None:
        data = trajectory()
        snapshot = make_mock_snapshot(data["start_state"])
        data["start_snapshot"] = snapshot
        data["start_snapshot_sha256"] = "not-a-valid-digest"
        with self.assertRaises(ValueError):
            validate_numeric(data)

    def test_import_requires_measured_mapping(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_json(root / "raw.json", {})
            write_json(root / "calibration.json", {"review_status": "unconfirmed"})
            with self.assertRaises(ValueError):
                import_recording(root / "raw.json", root / "calibration.json")

    def test_recording_mapping_is_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            evidence = root / "measurement.txt"
            evidence.write_text("SYNTHETIC UNIT TEST ONLY")
            raw = {"schema_version": 1, "kind": "arx_r5_sdk_recording", "robot_model": "ARX_R5",
                   "robot_serial": "TEST-ONLY", "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                   "sdk_commit": "test-sdk", "joint_positions": [1, 2, 3, 4, 5, 6, 7],
                   "joint_velocities": [0.1] * 7, "units": {"position": "rad", "velocity": "rad/s"}}
            calibration = {"review_status": "measured", "robot_model": "ARX_R5",
                           "robot_serial": "TEST-ONLY", "sdk_commit": "test-sdk",
                           "joint_names": load_profile()["joint_names"],
                           "sdk_indices": [5, 4, 3, 2, 1, 0], "sign": [-1, 1, 1, 1, 1, 1],
                           "offset_rad": [0.5, 0, 0, 0, 0, 0], "raw_units": raw["units"],
                           "evidence": {"path": "measurement.txt", "sha256": digest(evidence),
                                        "kind": "hardware_measurement"}}
            write_json(root / "raw.json", raw)
            write_json(root / "calibration.json", calibration)
            result = import_recording(root / "raw.json", root / "calibration.json")
            self.assertEqual(result["position"], [-5.5, 5, 4, 3, 2, 1])
            self.assertEqual(result["velocity"][0], -0.1)
            self.assertEqual(result["source"], "hardware_recording")
            self.assertFalse(result["hardware_eligible"])
            evidence.write_text("changed")
            with self.assertRaises(ValueError):
                import_recording(root / "raw.json", root / "calibration.json")


class PreflightTests(unittest.TestCase):
    def test_uncommissioned_profile_produces_actionable_blockers(self) -> None:
        data = trajectory()
        snapshot = make_mock_snapshot(data["start_state"])
        report = preflight(data, snapshot, ROOT / "config/commissioning.template.json")
        self.assertFalse(report["hardware_execution_allowed"])
        checks = {c["name"]: c for c in report["checks"]}
        self.assertEqual(checks["trajectory_numeric"]["status"], "pass")
        self.assertEqual(checks["current_state_source"]["status"], "fail")
        self.assertEqual(checks["hardware_backend"]["status"], "fail")
        self.assertEqual(checks["verified_dynamic_limits"]["status"], "unverified")

    def test_bad_goal_file_has_no_pass_report(self) -> None:
        data = trajectory()
        data["goal_state"][0] += 0.2
        report = preflight(data, make_mock_snapshot(data["start_state"]),
                           ROOT / "config/commissioning.template.json")
        self.assertFalse(report["hardware_execution_allowed"])
        self.assertEqual(report["checks"][0]["status"], "fail")
        for malformed in (None, [], "trajectory"):
            with self.subTest(malformed=malformed):
                report = preflight(malformed, {}, ROOT / "config/commissioning.template.json")
                self.assertEqual(report["checks"][0]["status"], "fail")
                self.assertFalse(report["hardware_execution_allowed"])

    def test_evidence_does_not_bypass_live_backend_and_numeric_checks(self) -> None:
        """合成完整证据只用于触发各检查分支，绝不写入生产标定文件。"""
        now = datetime.now(timezone.utc)
        current = make_mock_snapshot(load_profile()["default_position"])
        current.update({"source": "hardware_recording", "robot_serial": "SYNTHETIC-TEST",
                        "sdk_commit": "test-sdk", "calibration_sha256": "a" * 64,
                        "raw_recording_sha256": "b" * 64, "captured_at_utc": now.isoformat(),
                        "feedback_sequence": 1,
                        "feedback_health": {"communication_ok": True, "device_clock_verified": True,
                                            "device_sample_age_s": 0.01}})
        data = trajectory()
        bind_start_snapshot(data, current)
        profile = load_profile()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "evidence.txt").write_text("SYNTHETIC TEST DATA; not a hardware measurement")
            evidence = {"path": "evidence.txt", "sha256": digest(root / "evidence.txt"),
                        "kind": "hardware_measurement"}
            config = {"schema_version": 1,
                      "robot": {"model": "ARX_R5", "serial": "SYNTHETIC-TEST", "hardware_revision": "test",
                                "sdk_commit": "test-sdk", "calibration_sha256": "a" * 64},
                      "limits": {key: profile[key] for key in ("position_lower", "position_upper", "velocity", "acceleration")},
                      "model": {"provenance": provenance(), "locked_gripper": profile["locked_gripper"],
                                "minimum_clearance_m": 0.00001, "joint_limit_margin_rad": 0.01,
                                "fixtures_verified": True},
                      "controller": {"period_s": 0.025, "max_snapshot_age_s": 5.0,
                                     "start_tolerance_rad": 0.001, "stationary_velocity_rad_s": 0.001,
                                     "tracking_error_rad": 0.01, "max_lateness_s": 0.05,
                                     "max_feedback_age_s": 0.1, "interpolation_verified": True,
                                     "watchdog_verified": True, "feedback_freshness_verified": True,
                                     "host_clock_sync_verified": True},
                      "stop": {"procedure": "synthetic test", "acknowledgment_verified": True,
                               "communication_loss_verified": True, "estop_verified": True}}
            for key in ("robot", "limits", "model", "controller", "stop"):
                config[key]["evidence"] = evidence
            path = root / "commissioning.json"
            write_json(path, config)
            authorization = {"action": "execute_once", "authorized_by": "SYNTHETIC TEST",
                             "robot_serial": current["robot_serial"],
                             "trajectory_sha256": content_digest(data),
                             "current_snapshot_sha256": content_digest(current),
                             "issued_at_utc": (now - timedelta(seconds=1)).isoformat(),
                             "expires_at_utc": (now + timedelta(seconds=1)).isoformat()}
            report = preflight(data, current, path, authorization, now)
            checks = {c["name"]: c["status"] for c in report["checks"]}
            for name in ("robot_identity_and_mapping", "verified_dynamic_limits", "commissioned_geometry_and_margins",
                         "controller_timing_and_current_state", "acknowledged_safe_stop", "operator_authorization"):
                self.assertEqual(checks[name], "pass", report)
            self.assertFalse(report["hardware_execution_allowed"])
            mutations = [
                ("model", "minimum_clearance_m", 0.01, "commissioned_geometry_and_margins"),
                ("controller", "period_s", 0.01, "controller_timing_and_current_state"),
                ("limits", "velocity", [0.0] * 6, "verified_dynamic_limits"),
                ("robot", "serial", "DIFFERENT", "robot_identity_and_mapping"),
                ("stop", "acknowledgment_verified", False, "acknowledged_safe_stop"),
            ]
            for section, field, value, name in mutations:
                changed = copy.deepcopy(config)
                changed[section][field] = value
                write_json(path, changed)
                result = preflight(data, current, path, authorization, now)
                with self.subTest(field=field):
                    self.assertEqual(next(c for c in result["checks"] if c["name"] == name)["status"], "fail")
            write_json(path, config)
            for delta in (-10, 10):
                changed = copy.deepcopy(current)
                changed["captured_at_utc"] = (now + timedelta(seconds=delta)).isoformat()
                result = preflight(data, changed, path, authorization, now)
                checks = {c["name"]: c["status"] for c in result["checks"]}
                self.assertEqual(checks["controller_timing_and_current_state"], "fail")
                self.assertEqual(checks["operator_authorization"], "fail")
            changed = copy.deepcopy(current)
            changed["position"][0] += 0.05
            result = preflight(data, changed, path, authorization, now)
            checks = {c["name"]: c["status"] for c in result["checks"]}
            self.assertEqual(checks["controller_timing_and_current_state"], "fail")


class SchedulerTests(unittest.TestCase):
    def test_delayed_read_rejected_before_command(self) -> None:
        clock = VirtualClock()

        class SlowReader(MockController):
            def read_state(self, now: float) -> Feedback:
                clock.sleep(0.2)
                return super().read_state(now)

        controller = SlowReader(trajectory()["start_state"])
        with tempfile.TemporaryDirectory() as temp, self.assertRaisesRegex(RuntimeError, "Stale"):
            execute_mock(trajectory(), controller, Path(temp) / "slow.jsonl", clock=clock)
        self.assertEqual(controller.commands, 0)
        self.assertTrue(controller.stopped)

    def test_slow_send_never_bursts_to_catch_up(self) -> None:
        clock = VirtualClock()

        class SlowSender(MockController):
            def send(self, position: list[float], velocity: list[float], now: float) -> None:
                super().send(position, velocity, now)
                clock.sleep(0.2)

        controller = SlowSender(trajectory()["start_state"])
        with tempfile.TemporaryDirectory() as temp, self.assertRaises(TimeoutError):
            execute_mock(trajectory(), controller, Path(temp) / "slow.jsonl", clock=clock)
        self.assertEqual(controller.commands, 1)
        self.assertTrue(controller.stopped)

    def test_frozen_feedback_sequence(self) -> None:
        class FrozenReader(MockController):
            def read_state(self, now: float) -> Feedback:
                state = super().read_state(now)
                state.sequence = 0
                return state

        controller = FrozenReader(trajectory()["start_state"])
        with tempfile.TemporaryDirectory() as temp, self.assertRaisesRegex(RuntimeError, "sequence"):
            execute_mock(trajectory(), controller, Path(temp) / "frozen.jsonl")
        self.assertEqual(controller.commands, 0)

    def test_stop_failure_recorded_without_hiding_original_error(self) -> None:
        class BrokenStop(MockController):
            def stop(self) -> bool:
                raise ConnectionError("Synthetic stop failure")

        for fail_at in (None, 1):
            controller = BrokenStop(trajectory()["start_state"], fail_at=fail_at)
            with tempfile.TemporaryDirectory() as temp:
                path = Path(temp) / "stop.jsonl"
                expected = RuntimeError if fail_at is None else ConnectionError
                with self.assertRaises(expected):
                    execute_mock(trajectory(), controller, path)
                summary = json.loads(path.with_suffix(".summary.json").read_text())
                self.assertFalse(summary["mock_stop_confirmed"])
                self.assertIn("stop_error", summary)
                self.assertEqual(summary["status"], "stop_failed" if fail_at is None else "aborted")

    def test_failed_tracking_sample_is_in_error_statistics(self) -> None:
        controller = MockController(trajectory()["start_state"], offset=0.02)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "tracking.jsonl"
            with self.assertRaises(RuntimeError):
                execute_mock(trajectory(), controller, path)
            summary = json.loads(path.with_suffix(".summary.json").read_text())
            self.assertGreater(summary["max_error_rad"][0], 0.019)


if __name__ == "__main__":
    unittest.main()
