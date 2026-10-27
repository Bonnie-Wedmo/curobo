"""使用当前 cuRobo MotionPlanner.plan_cspace 进行六关节空间规划。"""
from __future__ import annotations

import argparse
from importlib.metadata import version
from pathlib import Path
from typing import Any

from arx_r5.common import JOINT_NAMES, ROOT, load_profile
from arx_r5.trajectory import export_trajectory, make_trajectory, vector


def plan(start: list[float], goal: list[float], output: Path,
         debug_eager: bool = False, start_snapshot: dict | None = None) -> dict[str, Any]:
    """仅返回通过数值和自碰撞复核的离线轨迹；不会连接 SDK。"""
    import torch
    import curobo
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    from curobo.types import JointState
    from arx_r5.planning.collision import SphereChecker
    from arx_r5.planning.model import robot_config

    profile = load_profile()
    snapshot_fields: dict[str, Any] = {"start_state": start}
    if start_snapshot is not None:
        from arx_r5.control.state import bind_start_snapshot
        bind_start_snapshot(snapshot_fields, start_snapshot)
    for name, q in [("start", start), ("goal", goal)]:
        vector(q, 6, name)
        if any(x < lo or x > hi for x, lo, hi in zip(
                q, profile["position_lower"], profile["position_upper"])):
            raise ValueError(f"{name} outside offline position limits")
    checker = SphereChecker()
    checker.validate([start])
    checker.validate([goal])
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable; use the existing .venv on a GPU-accessible host")
    cfg = MotionPlannerCfg.create(
        robot=robot_config(), scene_model=None, self_collision_check=True,
        num_ik_seeds=8, num_trajopt_seeds=2, use_cuda_graph=not debug_eager,
        interpolation_dt=profile["interpolation_dt"], interpolation_buffer_size=500,
    )
    with MotionPlanner(cfg) as planner:
        if planner.joint_names != JOINT_NAMES:
            raise ValueError(f"Unexpected planner joint order: {planner.joint_names}")
        # 六关节均显式指定；不经过 IK，以免改变关节空间终点。
        states = [JointState.from_position(torch.tensor([q], device="cuda", dtype=torch.float32),
                                           joint_names=JOINT_NAMES) for q in [start, goal]]
        result = planner.plan_cspace(goal_state=states[1], current_state=states[0])
        if result is None or result.success.numel() != 1 or not bool(result.success.item()):
            raise RuntimeError("Joint-space planning failed; no trajectory exported")
        # 该版本会把锁定夹爪补回完整轨迹，先按名字核对固定值，再提取六轴。
        trajectory = result.get_interpolated_plan()
        names = trajectory.joint_names
        expected = JOINT_NAMES + list(profile["locked_gripper"])
        if names is None or len(names) != len(set(names)) or set(names) != set(expected):
            raise ValueError(f"Unexpected interpolated joint names: {names}")
        for name, locked in profile["locked_gripper"].items():
            index = names.index(name)
            if not torch.allclose(trajectory.position[..., index],
                                  torch.full_like(trajectory.position[..., index], locked), atol=1e-6):
                raise ValueError("Locked gripper position changed")
            for field in ("velocity", "acceleration"):
                if torch.any(getattr(trajectory, field)[..., index].abs() > 1e-6):
                    raise ValueError("Locked gripper unexpectedly moving")
        trajectory = trajectory.reorder(JOINT_NAMES)
        values = []
        for name in ["position", "velocity", "acceleration"]:
            tensor = getattr(trajectory, name)
            if tensor is None or tensor.ndim != 4 or tuple(tensor.shape[:2]) != (1, 1):
                raise ValueError(f"Unsupported {name} shape; inspect this cuRobo version")
            values.append(tensor[0, 0].detach().cpu().tolist())
        # 当前结果没有 result.interpolation_dt；周期来自求解器配置，非 js_solution.dt。
        dt = float(planner.trajopt_solver.config.interpolation_dt)
        data = make_trajectory(start, goal, *values, dt)
        if start_snapshot is not None:
            data.update(snapshot_fields)
        data["planner"] = {"api": "MotionPlanner.plan_cspace", "version": version("nvidia-curobo"),
                           "module": curobo.__file__, "cuda_graph": not debug_eager,
                           "solve_time_s": float(result.solve_time),
                           "optimized_dt_s": result.js_solution.dt.detach().cpu().tolist()}
        # cuRobo 内部已经做约束检查，再用独立 CPU FK/球对检查导出数据。
        export_trajectory(output, data)
        return data


def main() -> None:
    """解析明确的关节测试状态并导出离线轨迹。"""
    parser = argparse.ArgumentParser(description=__doc__)
    start_group = parser.add_mutually_exclusive_group(required=True)
    start_group.add_argument("--start", type=float, nargs=6)
    start_group.add_argument("--start-snapshot", type=Path)
    parser.add_argument("--goal", type=float, nargs=6, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/trajectory.json")
    parser.add_argument("--debug-eager", action="store_true", help="仅用于 CUDA graph 调试")
    args = parser.parse_args()
    snapshot = None
    if args.start_snapshot is not None:
        from arx_r5.control.state import load_snapshot
        snapshot = load_snapshot(args.start_snapshot)
    start = snapshot["position"] if snapshot is not None else args.start
    data = plan(start, args.goal, args.output, args.debug_eager, snapshot)
    print(f"Offline planning passed: {len(data['time_from_start'])} samples -> {args.output}")
    print("Hardware eligible: False")


if __name__ == "__main__":
    main()
