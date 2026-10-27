"""逐轴、耦合与拒绝案例的离线验收矩阵；保留失败，不筛掉难例。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

from arx_r5.common import JOINT_NAMES, ROOT, content_digest, load_profile, provenance, write_json
from arx_r5.control.state import make_mock_snapshot
from arx_r5.planning.collision import SphereChecker


def cases(start: list[float]) -> list[dict]:
    """测试幅度是离线覆盖参数，绝不代表可在硬件上使用的运动范围。"""
    # 狭窄几何区也保留测试；大幅度失败后不会偷偷减小幅度以宣称通过。
    result = []
    for j, magnitude in enumerate([0.3, 0.03, 0.3, 0.15, 0.15, 0.03]):
        for sign in (-1, 1):
            delta = [0.0] * 6
            delta[j] = sign * magnitude
            result.append({"name": f"joint{j + 1}_{'negative' if sign < 0 else 'positive'}",
                           "delta_rad": delta})
    result += [
        {"name": "coupled_large", "delta_rad": [0.5, 0, 0.3, 0.15, -0.08, 0]},
        {"name": "coupled_reverse", "delta_rad": [-0.4, 0, -0.2, 0.1, -0.08, 0]},
        {"name": "joint2_large_positive", "delta_rad": [0, 0.3, 0, 0, 0, 0]},
        {"name": "joint6_large_positive", "delta_rad": [0, 0, 0, 0, 0, 0.3]},
    ]
    for case in result:
        case["start"] = start.copy()
        case["goal"] = [x + d for x, d in zip(start, case["delta_rad"])]
    return result


def run_suite(output_dir: Path, gpu: bool = False) -> dict:
    """端点无效即拒绝；有效端点可交给 GPU，最终再独立检查导出数据。"""
    profile = load_profile()
    checker = SphereChecker()
    start = profile["default_position"]
    snapshot = make_mock_snapshot(start)
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(output_dir / "start_snapshot.json", snapshot)
    report = {"created_at_utc": datetime.now(timezone.utc).isoformat(),
              "scope": "offline_test_only", "hardware_eligible": False,
              "provenance": provenance(), "gpu_requested": gpu, "cases": [],
              "note": "Coverage of these goals is not a model-wide safety or hardware acceptance claim"}
    for case in cases(start):
        entry = dict(case)
        entry["status"] = "not_run"
        try:
            lo, hi = profile["position_lower"], profile["position_upper"]
            entry["endpoint_position_margin_rad"] = min(min(x - a, b - x)
                for q in (start, case["goal"]) for x, a, b in zip(q, lo, hi))
            if entry["endpoint_position_margin_rad"] < 0:
                raise ValueError("Goal violates offline position bounds")
            entry["endpoint_clearance_m"] = [checker.clearance(q)[0] for q in (start, case["goal"])]
            if min(entry["endpoint_clearance_m"]) <= 0:
                raise ValueError("Endpoint is in model self-collision")
            try:
                entry["straight_segment_diagnostic"] = checker.validate([start, case["goal"]])
            except ValueError as error:
                # 直线不通不等于无法规划，保留 GPU 图搜索/优化机会。
                entry["straight_segment_diagnostic"] = {"passed": False, "error": str(error)}
            entry["status"] = "endpoints_valid_not_planned"
            if gpu:
                from arx_r5.planning.plan import plan
                path = output_dir / f"{case['name']}.json"
                data = plan(start, case["goal"], path, start_snapshot=snapshot)
                entry.update({"status": "gpu_planned_and_validated", "trajectory": path.name,
                              "trajectory_sha256": content_digest(data),
                              "samples": len(data["time_from_start"]),
                              "duration_s": data["time_from_start"][-1],
                              "collision": data["self_collision_validation"],
                              "max_goal_error_rad": max(abs(a - b) for a, b in
                                                       zip(data["joint_positions"][-1], case["goal"])),
                              "path_position_margin_rad": min(min(x - a, b - x) for q in
                                  data["joint_positions"] for x, a, b in zip(q, lo, hi))})
        except (ValueError, RuntimeError) as error:
            entry["status"] = "rejected"
            entry["reason"] = f"{type(error).__name__}: {error}"
        report["cases"].append(entry)
        # 每例落盘，长运行中断也保留实际进度，不把未完成案例计作成功。
        write_json(output_dir / "report.json", report)
        print(case["name"], entry["status"], entry.get("reason", ""), flush=True)
    report["summary"] = {"total": len(report["cases"]),
                         "gpu_passed": sum(c["status"] == "gpu_planned_and_validated" for c in report["cases"]),
                         "rejected": sum(c["status"] == "rejected" for c in report["cases"]),
                         "successful_requested_motion_rad": {
                             name: max((abs(c["delta_rad"][i]) for c in report["cases"]
                                        if c["status"] == "gpu_planned_and_validated"), default=0.0)
                             for i, name in enumerate(JOINT_NAMES)}}
    write_json(output_dir / "report.json", report)
    return report


def main() -> None:
    """--gpu 才真正规划；默认仅 CPU 端点/几何诊断。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gpu", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/deployment_suite")
    args = parser.parse_args()
    report = run_suite(args.output_dir, args.gpu)
    print(report["summary"])


if __name__ == "__main__":
    main()
