"""独立控制端：文件复核、Mock 调度、反馈记录及硬件阻塞诊断。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from arx_r5.common import ROOT, load_profile, write_json
from arx_r5.control.mock import Clock, Feedback, MockController, MonotonicClock, VirtualClock
from arx_r5.control.sdk import OfficialR5Adapter, SDK_CONTRACT
from arx_r5.trajectory import load_trajectory, validate_numeric, vector


def check_feedback(feedback: Feedback, now: float) -> None:
    """仅供 Mock 的演示阈值；正式 SDK 需要硬件时间戳和通信健康信息。"""
    vector(feedback.position, 6, "feedback position")
    vector(feedback.velocity, 6, "feedback velocity")
    if not math.isfinite(feedback.received_at) or not 0 <= now - feedback.received_at <= 0.1:
        raise RuntimeError("Stale or invalid feedback timestamp")
    if feedback.source != "mock":
        raise RuntimeError("Mock executor cannot accept a hardware controller")


def execute_mock(data: dict[str, Any], controller: MockController, log_path: Path,
                 realtime: bool = False, clock: Clock | None = None) -> dict[str, Any]:
    """按绝对时间调度（默认虚拟时钟），出现故障后禁止继续发送并记录停止结果。"""
    from arx_r5.planning.collision import SphereChecker

    clock = clock or (MonotonicClock() if realtime else VirtualClock())
    endpoint_tol = load_profile()["endpoint_tolerance"]
    log_path.parent.mkdir(parents=True, exist_ok=True)
    squared, maximum, samples = [0.0] * 6, [0.0] * 6, 0
    summary: dict[str, Any] = {"mode": "mock", "realtime": realtime, "hardware_test": False,
                               "status": "preflight", "commands": 0}
    with log_path.open("w") as log:
        def event(record: dict[str, Any]) -> None:
            log.write(json.dumps(record, allow_nan=False) + "\n")
            log.flush()

        last_sequence = -1

        def read_feedback() -> Feedback:
            nonlocal last_sequence
            feedback = controller.read_state(clock.now())
            # 调用可能阻塞：返回后重新读取时钟，不复用调用前的时间。
            check_feedback(feedback, clock.now())
            if type(feedback.sequence) is not int or feedback.sequence <= last_sequence:
                raise RuntimeError("Feedback acquisition sequence stalled or reversed")
            last_sequence = feedback.sequence
            return feedback

        try:
            validate_numeric(data)
            SphereChecker().validate(data["joint_positions"])
            feedback = read_feedback()
            if max(abs(a - b) for a, b in zip(feedback.position, data["start_state"])) > endpoint_tol:
                raise RuntimeError("Current position differs from planned start; replan required")
            if max(abs(v) for v in feedback.velocity) > 0.001:
                raise RuntimeError("Current state is moving; rest-to-rest plan required")
            epoch = clock.now()
            event({"event": "started", "mode": "mock", "clock": type(clock).__name__})
            for i, target_time in enumerate(data["time_from_start"]):
                deadline = epoch + target_time
                clock.sleep(max(0.0, deadline - clock.now()))
                if clock.now() - deadline > 0.1:
                    raise TimeoutError("Mock scheduler missed deadline")
                feedback = read_feedback()
                if clock.now() - deadline > 0.1:
                    raise TimeoutError("Feedback read exceeded command deadline; no command sent")
                controller.send(data["joint_positions"][i], data["joint_velocities"][i], clock.now())
                if clock.now() - deadline > 0.1:
                    raise TimeoutError("Command send exceeded deadline; abort without catch-up burst")
                feedback = read_feedback()
                now = clock.now()
                if now - deadline > 0.1:
                    raise TimeoutError("Post-command feedback exceeded deadline")
                error = [a - b for a, b in zip(feedback.position, data["joint_positions"][i])]
                event({"event": "feedback", "sample": i, "planned_time": target_time,
                       "actual_time": now - epoch, "command": data["joint_positions"][i],
                       "actual_position": feedback.position, "actual_velocity": feedback.velocity,
                       "error_rad": error, "source": "mock"})
                for j in range(6):
                    squared[j] += error[j] ** 2
                    maximum[j] = max(maximum[j], abs(error[j]))
                samples += 1
                if max(abs(x) for x in error) > 0.01:
                    raise RuntimeError("Mock tracking error exceeded demonstration tolerance")
            # 完成取决于末端反馈到达目标且静止，不仅仅是发完文件。
            if max(abs(a - b) for a, b in zip(feedback.position, data["goal_state"])) > endpoint_tol:
                raise RuntimeError("Final goal was not reached")
            if max(abs(x) for x in feedback.velocity) > 0.001:
                raise RuntimeError("Final state not stationary")
            summary["status"] = "completed"
        except BaseException as error:
            summary["status"] = "aborted"
            summary["error"] = f"{type(error).__name__}: {error}"
            raise
        finally:
            # 停止失败也必须落盘；不能因 finally 的异常覆盖最初故障。
            stop_error = None
            try:
                summary["mock_stop_confirmed"] = controller.stop() is True
                if not summary["mock_stop_confirmed"]:
                    raise RuntimeError("Mock stop returned no positive acknowledgment")
            except BaseException as error:
                stop_error = error
                summary["mock_stop_confirmed"] = False
                summary["stop_error"] = f"{type(error).__name__}: {error}"
                if summary["status"] == "completed":
                    summary["status"] = "stop_failed"
            summary["commands"] = controller.commands
            summary["rms_error_rad"] = [math.sqrt(x / samples) if samples else None for x in squared]
            summary["max_error_rad"] = maximum if samples else None
            event({"event": "finished", **summary})
            write_json(log_path.with_suffix(".summary.json"), summary)
            if stop_error is not None and summary["status"] == "stop_failed":
                raise RuntimeError("Mock safe-stop acknowledgment failed") from stop_error
    return summary


def main() -> None:
    """默认 Mock；hardware 模式只报告阻塞，不构造 SDK。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, nargs="?")
    parser.add_argument("--mode", choices=["mock", "hardware"], default="mock")
    parser.add_argument("--diagnose-sdk", action="store_true")
    parser.add_argument("--log", type=Path, default=ROOT / "artifacts/mock_execution.jsonl")
    initial_group = parser.add_mutually_exclusive_group()
    initial_group.add_argument("--initial", nargs=6, type=float)
    initial_group.add_argument("--initial-snapshot", type=Path)
    parser.add_argument("--fail-at", type=int)
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()
    if args.diagnose_sdk:
        print(json.dumps(SDK_CONTRACT | {"blockers": load_profile()["hardware_blockers"]},
                         ensure_ascii=False, indent=2))
        return
    try:
        if args.mode == "hardware":
            OfficialR5Adapter().initialize()
        if args.trajectory is None:
            parser.error("trajectory is required for mock execution")
        data = load_trajectory(args.trajectory)
        # 从文件起点初始化只属于 Mock；真机不得这样假定当前状态。
        initial = args.initial or data["start_state"]
        snapshot = None
        if args.initial_snapshot is not None:
            from arx_r5.control.state import load_snapshot
            snapshot = load_snapshot(args.initial_snapshot)
            if snapshot["source"] != "mock":
                raise ValueError("Mock controller initialization requires a Mock snapshot")
            initial = snapshot["position"]
        controller = MockController(initial, fail_at=args.fail_at)
        if snapshot is not None:
            controller.velocity = snapshot["velocity"].copy()
        print(execute_mock(data, controller, args.log, args.realtime))
    except (ValueError, RuntimeError, OSError) as error:
        parser.exit(2, f"Rejected: {error}\n")


if __name__ == "__main__":
    main()