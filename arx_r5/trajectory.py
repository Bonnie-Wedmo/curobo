"""标准库即可读取和验证的轨迹协议；控制端无需安装 cuRobo。"""
from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from arx_r5.common import JOINT_NAMES, content_digest, load_profile, provenance, write_json

UNITS = {"position": "rad", "velocity": "rad/s", "acceleration": "rad/s^2", "time": "s"}


def vector(value: Any, size: int, field: str) -> list[float]:
    """严格检查有限实数向量，拒绝布尔、字符串及长度不匹配。"""
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{field}: expected {size} values")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError(f"{field}: non-finite or non-numeric values")
    return value


def validate_numeric(data: dict[str, Any]) -> dict[str, Any]:
    """检查协议、来源、端点、限位及差分连续性；不把离线限制称为实机限制。"""
    if not isinstance(data, dict):
        raise ValueError("Trajectory must be a JSON object")
    p = load_profile()
    if data.get("schema_version") != 1 or data.get("robot_model") != p["robot_model"]:
        raise ValueError("Unknown schema or robot model")
    if data.get("joint_names") != JOINT_NAMES or data.get("units") != UNITS:
        raise ValueError("Joint order or units mismatch")
    if data.get("configuration_version") != p["configuration_version"]:
        raise ValueError("Configuration version mismatch")
    if data.get("provenance") != provenance():
        raise ValueError("Model/configuration hash mismatch")
    if data.get("hardware_eligible") is not False:
        raise ValueError("This module cannot certify hardware eligibility")
    if data.get("planning_status") != "success":
        raise ValueError("Planner did not succeed")
    if data.get("start_state_source") not in ("offline_test", "hardware_snapshot_unverified"):
        raise ValueError("Missing start-state provenance")
    if data.get("locked_gripper") != p["locked_gripper"]:
        raise ValueError("Gripper configuration mismatch")
    start = vector(data.get("start_state"), 6, "start_state")
    goal = vector(data.get("goal_state"), 6, "goal_state")
    if data.get("start_snapshot") is not None:
        from arx_r5.control.state import bind_start_snapshot
        snapshot = data["start_snapshot"]
        if content_digest(snapshot) != data.get("start_snapshot_sha256"):
            raise ValueError("Start snapshot content hash mismatch")
        check = {"start_state": start}
        bind_start_snapshot(check, snapshot)
        if check["start_state_source"] != data["start_state_source"]:
            raise ValueError("Start state source does not agree with bound snapshot")
    elif data["start_state_source"] == "hardware_snapshot_unverified":
        raise ValueError("Hardware-derived start requires a bound state snapshot")
    times = data.get("time_from_start")
    if not isinstance(times, list) or not 3 <= len(times) <= 100000:
        raise ValueError("Expected 3..100000 timestamps")
    vector(times, len(times), "time_from_start")
    if times[0] != 0.0 or any(b <= a for a, b in zip(times[:-1], times[1:])):
        raise ValueError("Timestamps must start at zero and strictly increase")
    interpolation_dt = data.get("interpolation_dt")
    if type(interpolation_dt) not in (float, int) or not math.isfinite(interpolation_dt) or interpolation_dt <= 0:
        raise ValueError("Invalid interpolation_dt")
    if any(abs((b - a) - interpolation_dt) > 1e-8 for a, b in zip(times[:-1], times[1:])):
        raise ValueError("Timestamps disagree with interpolation_dt")
    arrays = []
    for key in ["joint_positions", "joint_velocities", "joint_accelerations"]:
        values = data.get(key)
        if not isinstance(values, list) or len(values) != len(times):
            raise ValueError(f"{key}: time dimension mismatch")
        arrays.append([vector(row, 6, key) for row in values])
    positions, velocities, accelerations = arrays
    tolerance = p["numeric_tolerance"]
    endpoint = p["endpoint_tolerance"]
    if max(abs(a - b) for a, b in zip(positions[0], start)) > endpoint:
        raise ValueError("Start state mismatch")
    if max(abs(a - b) for a, b in zip(positions[-1], goal)) > endpoint:
        raise ValueError("Goal state mismatch")
    for row in [start, goal] + positions:
        if any(x < lo - tolerance or x > hi + tolerance for x, lo, hi in
               zip(row, p["position_lower"], p["position_upper"])):
            raise ValueError("Joint position limit violated")
    for key, values in [("velocity", velocities), ("acceleration", accelerations)]:
        if any(abs(x) > limit + tolerance for row in values for x, limit in zip(row, p[key])):
            raise ValueError(f"Joint {key} limit violated")
    # 位置差分独立于文件内的速度字段，防止以全零速度伪装关节跳跃。
    slopes = []
    for i, (t0, t1) in enumerate(zip(times[:-1], times[1:])):
        dt = t1 - t0
        slope = [(b - a) / dt for a, b in zip(positions[i], positions[i + 1])]
        slopes.append(slope)
        for j in range(6):
            vmax, amax = p["velocity"][j], p["acceleration"][j]
            if abs(slope[j]) > vmax + tolerance:
                raise ValueError("Position discontinuity / finite-difference velocity")
            if abs(velocities[i + 1][j] - velocities[i][j]) > amax * dt + tolerance:
                raise ValueError("Velocity discontinuity / finite-difference acceleration")
            expected_delta = (velocities[i][j] + velocities[i + 1][j]) * dt * 0.5
            actual_delta = positions[i + 1][j] - positions[i][j]
            if abs(actual_delta - expected_delta) > amax * dt * dt + tolerance:
                raise ValueError("Position and velocity are inconsistent")
            expected_dv = (accelerations[i][j] + accelerations[i + 1][j]) * dt * 0.5
            if abs(velocities[i + 1][j] - velocities[i][j] - expected_dv) > p["jerk"] * dt**2 + tolerance:
                raise ValueError("Velocity and acceleration are inconsistent")
    for i in range(1, len(slopes)):
        dt_mid = (times[i + 1] - times[i - 1]) * 0.5
        if any(abs(b - a) > limit * dt_mid + 2 * tolerance / min(
                times[i] - times[i - 1], times[i + 1] - times[i])
               for a, b, limit in zip(slopes[i - 1], slopes[i], p["acceleration"])):
            raise ValueError("Position second difference exceeds acceleration bound")
    if max(abs(x) for x in velocities[0] + velocities[-1]) > 0.001:
        raise ValueError("Rest-to-rest trajectory required")
    return {"passed": True, "scope": "offline_profile_only", "hardware_limits_verified": False,
            "position_source": p["position_source"], "dynamics_source": p["dynamics_source"],
            "max_abs_velocity": [max(abs(row[j]) for row in velocities) for j in range(6)],
            "max_abs_acceleration": [max(abs(row[j]) for row in accelerations) for j in range(6)]}


def make_trajectory(start: list[float], goal: list[float], positions: list[list[float]],
                    velocities: list[list[float]], accelerations: list[list[float]], dt: float,
                    start_source: str = "offline_test") -> dict[str, Any]:
    """时间戳来自插值周期，绝不使用优化步长或假定的控制器周期。"""
    p = load_profile()
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("Invalid interpolation dt")
    return {"schema_version": 1, "robot_model": p["robot_model"], "joint_names": JOINT_NAMES,
            "start_state": start, "goal_state": goal, "start_state_source": start_source,
            "time_from_start": [i * dt for i in range(len(positions))],
            "joint_positions": positions, "joint_velocities": velocities,
            "joint_accelerations": accelerations, "planning_status": "success",
            "configuration_version": p["configuration_version"], "provenance": provenance(),
            "created_at_utc": datetime.now(timezone.utc).isoformat(), "units": UNITS,
            "interpolation_dt": dt, "controller_period": None,
            "locked_gripper": p["locked_gripper"], "hardware_eligible": False,
            "hardware_blockers": p["hardware_blockers"],
            "joint_limits_validation": {"passed": False},
            "self_collision_validation": {"passed": False}}


def load_trajectory(path: Path) -> dict[str, Any]:
    """标准 JSON 读取并重做数值检查；碰撞验证由规划/执行入口另行执行。"""
    if path.stat().st_size > 100_000_000:
        raise ValueError("Trajectory file too large")
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError("Trajectory must be a JSON object")
    validate_numeric(data)
    return data


def export_trajectory(path: Path, data: dict[str, Any]) -> None:
    """导出前重新检查数值及几何；失败不写出轨迹文件。"""
    from arx_r5.planning.collision import SphereChecker

    data["joint_limits_validation"] = validate_numeric(data)
    data["self_collision_validation"] = SphereChecker().validate(data["joint_positions"])
    write_json(path, data)


def main() -> None:
    """独立读取入口：python -m arx_r5.trajectory FILE。"""
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    data = load_trajectory(args.path)
    print(json.dumps({"samples": len(data["time_from_start"]), "units": data["units"],
                      "numeric_validation": "passed", "collision_rechecked": False,
                      "hardware_eligible": False}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
