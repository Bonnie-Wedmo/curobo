"""跨机状态快照与已有 SDK 记录导入；不创建 SDK 对象或连接 CAN。"""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from arx_r5.common import JOINT_NAMES, content_digest, digest, load_profile, write_json
from arx_r5.trajectory import vector


def utc_time(value: Any) -> datetime:
    """解析带时区的采集时间；不混用不同电脑的单调时钟。"""
    if not isinstance(value, str):
        raise ValueError("UTC timestamp is required")
    try:
        result = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError("Invalid timestamp") from error
    if result.tzinfo is None or result.utcoffset() is None:
        raise ValueError("Timestamp must include timezone")
    return result.astimezone(timezone.utc)


def verify_evidence(evidence: Any, directory: Path) -> None:
    """校验证据文件存在且摘要一致；这不是对文件内容真实性的认证。"""
    if not isinstance(evidence, dict) or evidence.get("kind") not in (
            "hardware_measurement", "manufacturer_document"):
        raise ValueError("Manufacturer document or hardware measurement evidence is required")
    if not isinstance(evidence.get("path"), str) or not evidence["path"].strip():
        raise ValueError("Evidence path missing")
    path = directory / evidence["path"]
    if not path.is_file() or digest(path) != evidence.get("sha256"):
        raise ValueError("Evidence file absent or SHA256 mismatch")


def validate_snapshot(data: Any) -> None:
    """验证归一化六轴快照；来源是记录声明，不能当成现场新鲜反馈。"""
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("Unsupported state snapshot schema")
    if data.get("kind") != "arx_r5_joint_snapshot" or data.get("robot_model") != "ARX_R5":
        raise ValueError("State snapshot robot or kind mismatch")
    if data.get("source") not in ("mock", "hardware_recording"):
        raise ValueError("Unrecognized state source")
    if data.get("joint_names") != JOINT_NAMES:
        raise ValueError("Snapshot joint order mismatch")
    if data.get("units") != {"position": "rad", "velocity": "rad/s"}:
        raise ValueError("Snapshot units mismatch")
    if data.get("hardware_eligible") is not False:
        raise ValueError("A snapshot cannot certify hardware execution eligibility")
    if not isinstance(data.get("robot_serial"), str) or not data["robot_serial"].strip():
        raise ValueError("Robot serial or explicit mock identifier required")
    vector(data.get("position"), 6, "snapshot position")
    vector(data.get("velocity"), 6, "snapshot velocity")
    utc_time(data.get("captured_at_utc"))
    if data["source"] == "hardware_recording":
        for name in ("calibration_sha256", "raw_recording_sha256"):
            value = data.get(name)
            if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
                raise ValueError(f"Invalid {name}")
        if data.get("physical_state_verified") is not False:
            raise ValueError("Imported recordings must remain physically unverified")


def make_mock_snapshot(position: list[float], velocity: list[float] | None = None) -> dict:
    """显式生成 Mock 快照，不提供将其改标为真实采样的选项。"""
    result = {"schema_version": 1, "kind": "arx_r5_joint_snapshot", "source": "mock",
              "robot_model": "ARX_R5", "robot_serial": "MOCK-ONLY", "joint_names": JOINT_NAMES,
              "position": list(vector(position, 6, "position")),
              "velocity": list(vector(velocity if velocity is not None else [0.0] * 6, 6, "velocity")),
              "units": {"position": "rad", "velocity": "rad/s"},
              "captured_at_utc": datetime.now(timezone.utc).isoformat(),
              "gripper_model_positions": load_profile()["locked_gripper"],
              "hardware_eligible": False, "physical_state_verified": False,
              "feedback_health": {"source": "mock"}}
    validate_snapshot(result)
    return result


def load_snapshot(path: Path) -> dict:
    """读取结构化快照；不因文件存在就认定机器人当前仍在该状态。"""
    data = json.loads(path.read_text())
    validate_snapshot(data)
    return data


def import_recording(recording_path: Path, calibration_path: Path) -> dict:
    """导入现场程序已有的七维 SDK 记录，按明确标定转换成 URDF 六轴顺序。"""
    raw = json.loads(recording_path.read_text())
    calibration = json.loads(calibration_path.read_text())
    if not isinstance(calibration, dict) or calibration.get("review_status") != "measured":
        raise ValueError("Explicit measured joint mapping required; no identity default")
    verify_evidence(calibration.get("evidence"), calibration_path.parent)
    if not isinstance(raw, dict) or raw.get("schema_version") != 1 or raw.get("kind") != "arx_r5_sdk_recording":
        raise ValueError("Unsupported raw SDK recording")
    if calibration.get("robot_model") != "ARX_R5" or calibration.get("joint_names") != JOINT_NAMES:
        raise ValueError("Invalid calibration model/joint order")
    for key in ("robot_model", "robot_serial", "sdk_commit"):
        if not raw.get(key) or raw.get(key) != calibration.get(key):
            raise ValueError(f"Recording/calibration {key} mismatch")
    positions = vector(raw.get("joint_positions"), 7, "SDK positions")
    velocities = vector(raw.get("joint_velocities"), 7, "SDK velocities")
    indices = calibration.get("sdk_indices")
    if not isinstance(indices, list) or any(type(i) is not int for i in indices) or sorted(indices) != list(range(6)):
        raise ValueError("sdk_indices must be a permutation of 0..5; index 6 is the gripper")
    signs = vector(calibration.get("sign"), 6, "sign")
    if any(s not in (-1, 1) for s in signs):
        raise ValueError("Joint signs must be +1 or -1")
    offsets = vector(calibration.get("offset_rad"), 6, "offset_rad")
    units = raw.get("units")
    if units != calibration.get("raw_units") or units not in (
            {"position": "rad", "velocity": "rad/s"},
            {"position": "deg", "velocity": "deg/s"}):
        raise ValueError("Explicit consistent raw position/velocity units required")
    scale = 1.0 if units["position"] == "rad" else math.pi / 180.0
    # q_model[i] = sign[i] * unit_scale * q_sdk[index[i]] + offset_rad[i]。
    result = {"schema_version": 1, "kind": "arx_r5_joint_snapshot", "source": "hardware_recording",
              "robot_model": "ARX_R5", "robot_serial": raw["robot_serial"],
              "sdk_commit": raw["sdk_commit"], "joint_names": JOINT_NAMES,
              "position": [signs[i] * scale * positions[j] + offsets[i] for i, j in enumerate(indices)],
              "velocity": [signs[i] * scale * velocities[j] for i, j in enumerate(indices)],
              "units": {"position": "rad", "velocity": "rad/s"},
              "captured_at_utc": raw.get("captured_at_utc"),
              "calibration_sha256": digest(calibration_path),
              "raw_recording_sha256": digest(recording_path),
              "gripper_raw": {"position": positions[6], "velocity": velocities[6]},
              "gripper_model_positions": raw.get("gripper_model_positions"),
              "feedback_health": raw.get("feedback_health"),
              "device_timestamp": raw.get("device_timestamp"),
              "feedback_sequence": raw.get("feedback_sequence"),
              "hardware_eligible": False, "physical_state_verified": False}
    validate_snapshot(result)
    return result


def bind_start_snapshot(trajectory: dict, snapshot: dict) -> None:
    """绑定完整快照及其内容摘要，防止只携带六个角度而丢失采集来源。"""
    validate_snapshot(snapshot)
    if snapshot["position"] != trajectory["start_state"]:
        raise ValueError("Snapshot position must exactly match requested start")
    if max(abs(v) for v in snapshot["velocity"]) > 0.001:
        raise ValueError("Rest-to-rest planning rejects a moving snapshot (offline threshold)")
    if snapshot.get("gripper_model_positions") != load_profile()["locked_gripper"]:
        raise ValueError("Snapshot gripper geometry does not match fixed planning model")
    trajectory["start_snapshot"] = snapshot
    trajectory["start_snapshot_sha256"] = content_digest(snapshot)
    trajectory["start_state_source"] = ("offline_test" if snapshot["source"] == "mock"
                                        else "hardware_snapshot_unverified")


def main() -> None:
    """生成 Mock 快照或导入现成记录；不会自动启动 SDK。"""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    mock = sub.add_parser("mock")
    mock.add_argument("--position", type=float, nargs=6, required=True)
    mock.add_argument("--output", type=Path, required=True)
    imp = sub.add_parser("import")
    imp.add_argument("--recording", type=Path, required=True)
    imp.add_argument("--calibration", type=Path, required=True)
    imp.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        data = (make_mock_snapshot(args.position) if args.command == "mock"
                else import_recording(args.recording, args.calibration))
        write_json(args.output, data)
        print(f"Snapshot exported: {args.output}; source={data['source']}; hardware_eligible=False")
    except (ValueError, OSError) as error:
        parser.exit(2, f"Rejected: {error}\n")


if __name__ == "__main__":
    main()
