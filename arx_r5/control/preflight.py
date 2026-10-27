"""不连接设备的部署预检：逐项报告已检查、失败与尚无证据的条件。"""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from arx_r5.common import ROOT, content_digest, provenance, write_json
from arx_r5.control.state import utc_time, validate_snapshot, verify_evidence
from arx_r5.trajectory import validate_numeric, vector


def positive(value: Any, name: str) -> float:
    """拒绝空值、布尔及未经明确填写的阈值。"""
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a documented positive finite value")
    return float(value)


def preflight(data: dict, current: dict, commissioning_path: Path,
              authorization: dict | None = None, now: datetime | None = None) -> dict:
    """依据控制机自己的现场配置复核文件；任何报告都不会启用硬件。"""
    now = now or datetime.now(timezone.utc)
    checks: list[dict] = []
    report = {"kind": "arx_r5_deployment_preflight", "checked_at_utc": now.isoformat(),
              "hardware_execution_allowed": False, "checks": checks,
              "note": "Read-only diagnostic; evidence hashes do not certify authenticity or enable SDK"}

    def record(name: str, status: str, detail: Any) -> None:
        checks.append({"name": name, "status": status, "detail": detail})

    def check(name: str, action: Callable[[], Any]) -> bool:
        try:
            detail = action()
            record(name, "pass", detail)
            return True
        except (ValueError, KeyError, TypeError, OSError) as error:
            record(name, "fail", str(error))
            return False

    def require(condition: bool, detail: str) -> None:
        if not condition:
            raise ValueError(detail)

    # 硬件 backend 与现场采样尚未验收，不允许证据文件的布尔值解除硬阻塞。
    backend_failure = "Live SDK acquisition, timed tracking and acknowledged safe stop are not implemented/commissioned"
    if not check("trajectory_numeric", lambda: validate_numeric(data)):
        record("hardware_backend", "fail", backend_failure)
        return report
    report["trajectory_sha256"] = content_digest(data)
    if not check("snapshot_structure", lambda: validate_snapshot(current)):
        record("hardware_backend", "fail", backend_failure)
        return report
    report["current_snapshot_sha256"] = content_digest(current)
    record("current_state_source", "fail",
           f"{current['source']} is not verified live hardware feedback; file age alone is insufficient")
    check("planned_start_source", lambda: require(
        data.get("start_state_source") == "hardware_snapshot_unverified" and "start_snapshot" in data,
        "Planned start is test data or has no bound hardware recording"))
    try:
        config = json.loads(commissioning_path.read_text())
        if not isinstance(config, dict) or config.get("schema_version") != 1:
            raise ValueError("Unsupported commissioning schema")
    except (ValueError, OSError) as error:
        record("commissioning_configuration", "fail", str(error))
        record("hardware_backend", "fail", backend_failure)
        return report

    def evidence_section(name: str, check_name: str, action: Callable[[dict], Any]) -> bool:
        section = config.get(name)
        if not isinstance(section, dict) or not section.get("evidence"):
            record(check_name, "unverified", f"Commissioning section {name} lacks evidence")
            return False

        def evaluate() -> Any:
            verify_evidence(section["evidence"], commissioning_path.parent)
            return action(section)

        return check(check_name, evaluate)

    def identity(section: dict) -> dict:
        require(section.get("model") == "ARX_R5", "Robot model must be exactly ARX_R5")
        require(bool(section.get("serial")) and bool(section.get("hardware_revision")),
                "Hardware serial/revision must be identified")
        require(section["serial"] == current["robot_serial"], "Current state is from another robot")
        planned = data.get("start_snapshot", {})
        require(planned.get("robot_serial") == current["robot_serial"], "Planned/current robot serial mismatch")
        require(bool(section.get("sdk_commit")) and section["sdk_commit"] == current.get("sdk_commit"),
                "SDK version mismatch")
        require(bool(section.get("calibration_sha256")) and section["calibration_sha256"] ==
                current.get("calibration_sha256") == planned.get("calibration_sha256"),
                "Joint calibration mismatch")
        return {"robot_serial": current["robot_serial"]}

    evidence_section("robot", "robot_identity_and_mapping", identity)

    def limits(section: dict) -> dict:
        lo = vector(section.get("position_lower"), 6, "hardware position_lower")
        hi = vector(section.get("position_upper"), 6, "hardware position_upper")
        velocity = vector(section.get("velocity"), 6, "hardware velocity")
        acceleration = vector(section.get("acceleration"), 6, "hardware acceleration")
        require(all(a < b for a, b in zip(lo, hi)), "Invalid hardware position range")
        for value in velocity + acceleration:
            positive(value, "hardware dynamics limit")
        rows = data["joint_positions"] + [current["position"]]
        require(all(a <= x <= b for row in rows for x, a, b in zip(row, lo, hi)),
                "Hardware position limit exceeded")
        for field, maximum in [("joint_velocities", velocity), ("joint_accelerations", acceleration)]:
            require(all(abs(x) <= bound for row in data[field] for x, bound in zip(row, maximum)),
                    f"Hardware {field} limit exceeded")
        dt = data["interpolation_dt"]
        for a, b, va, vb in zip(data["joint_positions"][:-1], data["joint_positions"][1:],
                                data["joint_velocities"][:-1], data["joint_velocities"][1:]):
            require(all(abs(y - x) <= bound * dt for x, y, bound in zip(a, b, velocity)),
                    "Hardware finite-difference velocity limit exceeded")
            require(all(abs(y - x) <= bound * dt for x, y, bound in zip(va, vb, acceleration)),
                    "Hardware finite-difference acceleration limit exceeded")
        return {"minimum_position_margin_rad": min(min(x - a, b - x)
                for row in rows for x, a, b in zip(row, lo, hi))}

    limits_valid = evidence_section("limits", "verified_dynamic_limits", limits)

    collision_report: dict = {}

    def collision() -> dict:
        from arx_r5.planning.collision import SphereChecker
        collision_report.update(SphereChecker().validate(data["joint_positions"]))
        return collision_report

    collision_valid = check("independent_model_collision", collision)

    def model(section: dict) -> dict:
        require(section.get("provenance") == provenance() == data["provenance"], "Model hashes differ")
        require(section.get("locked_gripper") == current.get("gripper_model_positions") ==
                data["locked_gripper"], "Actual gripper model state unknown or different")
        require(section.get("fixtures_verified") is True, "Base fixture/payload geometry not verified")
        clearance = positive(section.get("minimum_clearance_m"), "minimum_clearance_m")
        margin = positive(section.get("joint_limit_margin_rad"), "joint_limit_margin_rad")
        require(collision_valid and collision_report["minimum_clearance_m"] >= clearance,
                "Collision clearance is below commissioned minimum")
        require(limits_valid, "Verified hardware position limits are required for margin check")
        require(all(min(x - a, b - x) >= margin for row in data["joint_positions"] for x, a, b in
                    zip(row, config["limits"]["position_lower"], config["limits"]["position_upper"])),
                "Joint limit margin is below commissioned minimum")
        return {"minimum_clearance_m": collision_report["minimum_clearance_m"],
                "continuous_collision_proof": False}

    evidence_section("model", "commissioned_geometry_and_margins", model)

    def controller(section: dict) -> dict:
        values = {key: positive(section.get(key), key) for key in (
            "period_s", "max_snapshot_age_s", "start_tolerance_rad", "stationary_velocity_rad_s",
            "tracking_error_rad", "max_lateness_s", "max_feedback_age_s")}
        require(abs(values["period_s"] - data["interpolation_dt"]) <= 1e-9,
                "Controller period differs from trajectory; resample/revalidate on planner, do not replay as-is")
        for flag in ("interpolation_verified", "watchdog_verified", "feedback_freshness_verified",
                     "host_clock_sync_verified"):
            require(section.get(flag) is True, f"Controller {flag} not confirmed")
        age = (now - utc_time(current["captured_at_utc"])).total_seconds()
        require(0 <= age <= values["max_snapshot_age_s"], "Current snapshot stale or timestamp in future")
        error = max(abs(a - b) for a, b in zip(current["position"], data["start_state"]))
        require(error <= values["start_tolerance_rad"], "Start mismatch; replan, do not jump to first waypoint")
        require(max(abs(v) for v in current["velocity"]) <= values["stationary_velocity_rad_s"],
                "Robot is not stationary")
        health = current.get("feedback_health")
        require(isinstance(health, dict) and health.get("communication_ok") is True and
                health.get("device_clock_verified") is True, "Feedback device freshness/health unavailable")
        sample_age = health.get("device_sample_age_s")
        require(type(sample_age) in (int, float) and math.isfinite(sample_age) and
                0 <= sample_age <= values["max_feedback_age_s"], "Device sample age invalid or stale")
        sequence = current.get("feedback_sequence")
        require(type(sequence) is int and sequence >= 0, "Hardware acquisition sequence missing")
        return {"snapshot_age_s": age, "start_error_rad": error}

    evidence_section("controller", "controller_timing_and_current_state", controller)

    def stopping(section: dict) -> None:
        require(isinstance(section.get("procedure"), str) and bool(section["procedure"].strip()),
                "Safe-stop procedure missing")
        for flag in ("acknowledgment_verified", "communication_loss_verified", "estop_verified"):
            require(section.get(flag) is True, f"Stop {flag} not confirmed")

    evidence_section("stop", "acknowledged_safe_stop", stopping)
    if authorization is None:
        record("operator_authorization", "unverified", "No authorization for this trajectory/current state")
    else:
        def authorize() -> None:
            require(isinstance(authorization, dict), "Authorization must be a JSON object")
            require(authorization.get("action") == "execute_once", "Wrong authorization action")
            require(bool(authorization.get("authorized_by")), "Operator identity missing")
            require(authorization.get("robot_serial") == current["robot_serial"], "Authorization robot mismatch")
            require(authorization.get("trajectory_sha256") == content_digest(data), "Authorization trajectory mismatch")
            require(authorization.get("current_snapshot_sha256") == content_digest(current), "Authorization state mismatch")
            require(utc_time(authorization.get("issued_at_utc")) <= now <
                    utc_time(authorization.get("expires_at_utc")), "Authorization expired or not yet valid")
        check("operator_authorization", authorize)
    record("trajectory_hardware_qualification", "fail", "Trajectory was generated with an uncommissioned offline profile")
    record("hardware_backend", "fail", backend_failure)
    return report


def main() -> None:
    """将预检报告写盘；被阻塞时退出码为 2，但不会调用硬件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--current-state", type=Path, required=True)
    parser.add_argument("--commissioning", type=Path, default=ROOT / "config/commissioning.template.json")
    parser.add_argument("--authorization", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/deployment_preflight.json")
    args = parser.parse_args()
    try:
        data = json.loads(args.trajectory.read_text())
        current = json.loads(args.current_state.read_text())
        authorization = json.loads(args.authorization.read_text()) if args.authorization else None
        report = preflight(data, current, args.commissioning, authorization)
        write_json(args.output, report)
        for item in report["checks"]:
            print(f"{item['status']:10} {item['name']}: {item['detail']}")
        parser.exit(2, f"Hardware execution blocked. Report: {args.output}\n")
    except (ValueError, OSError) as error:
        parser.exit(2, f"Invalid preflight input: {error}\n")


if __name__ == "__main__":
    main()
