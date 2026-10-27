"""复用 demo 的规划和 Viser 流程，离线规划 JSON 中的三个抓取运动阶段。"""
from __future__ import annotations

import argparse
import json
import time
from importlib.metadata import version
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import ContentPath, GoalToolPose, JointState, ToolPoseCriteria
from curobo.viewer import ViserVisualizer

from arx_r5.common import JOINT_NAMES, ROOT, content_digest, load_profile
from arx_r5.planning.model import forward_kinematics, robot_config
from arx_r5.trajectory import export_trajectory, make_trajectory, validate_numeric, vector

POSITION_TOLERANCE_M = 0.003
ROTATION_TOLERANCE_RAD = 0.02


def transform(value: object, name: str) -> np.ndarray:
    """读取米制刚体变换 A_T_B；拒绝非有限值及非旋转矩阵。"""
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError(f"{name}: expected a 4x4 transform")
    for row in value:
        vector(row, 4, name)
    result = np.asarray(value, dtype=np.float32)
    rotation = result[:3, :3]
    if (not np.isfinite(result).all()
            or not np.allclose(result[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)):
        raise ValueError(f"{name}: invalid rigid transform")
    return result


def load_request(path: Path) -> tuple[dict, list[float], list[tuple[str, np.ndarray]]]:
    """按用户确认的恒等映射读取六轴起点和基座下的 link6 目标。

    目标已包含工具偏移转换，不再使用 sdk_ee_T_grasp_geometry 做任何乘法。
    sdk_reference_T_ee_at_capture 仅为采集记录，不用它重定义起点或求映射。
    """
    request = json.loads(path.read_text())
    if (not isinstance(request, dict) or request.get("schema_version") != 1
            or request.get("data_kind") != "grasp_plan_request"
            or request.get("joint_names") != JOINT_NAMES):
        raise ValueError("Unsupported grasp request schema or joint order")
    raw_start = vector(request["start_joint_positions_rad"], 6, "start_joint_positions_rad")
    task = request["task"]
    if not isinstance(task, dict):
        raise ValueError("task must be a JSON object")
    if (task.get("reference_frame") != "sdk_cartesian_reference"
            or task.get("controlled_frame") != "sdk_controlled_ee"
            or task.get("quaternion_order") != "wxyz"
            or task.get("units", {}).get("translation") != "m"
            or task.get("units", {}).get("angle") != "rad"):
        raise ValueError("Unexpected SDK frame or units")
    names = [stage["target"] for stage in task["stages"]
             if stage["action"] == "plan_move_and_confirm_arrival"]
    if any(stage["action"] not in ("plan_move_and_confirm_arrival", "set_catch_pos")
           for stage in task["stages"]):
        raise ValueError("Unsupported grasp stage action")
    if names != ["pregrasp", "grasp", "lift"]:
        raise ValueError("Expected ordered pregrasp, grasp and lift motion stages")
    targets = [(name, transform(task["targets"][name]["sdk_reference_T_ee_target"], name))
               for name in names]
    return request, raw_start, targets


def linear_axis(start: np.ndarray, goal: np.ndarray) -> tuple[str, bool]:
    """选取 cuRobo 直线运动轴及误差参考系：目标工具坐标或模型基座坐标。"""
    if Rotation.from_matrix(start[:3, :3].T @ goal[:3, :3]).magnitude() > ROTATION_TOLERANCE_RAD:
        raise ValueError("Linear stage requires the same start and goal orientation")
    delta = goal[:3, 3] - start[:3, 3]
    # grasp 沿工具 X；lift 沿基座 Z。均由实际请求位姿推导，不重算偏移。
    for direction, project in ((goal[:3, :3].T @ delta, True), (delta, False)):
        axis = int(np.argmax(np.abs(direction)))
        if np.linalg.norm(np.delete(direction, axis)) < 1e-6:
            return "xyz"[axis], project
    raise ValueError("Linear stage direction must align with a tool or base axis")


def validate_pose_path(poses: np.ndarray, start: np.ndarray, goal: np.ndarray,
                       linear: bool) -> dict[str, float]:
    """复核采样 FK 的终点和直线/定姿约束；超差拒绝，不把优化代价当保证。"""
    points = poses[:, :3, 3]
    position_error = float(np.linalg.norm(points[-1] - goal[:3, 3]))
    angle_errors = Rotation.from_matrix(goal[:3, :3].T @ poses[:, :3, :3]).magnitude()
    chord = goal[:3, 3] - start[:3, 3]
    fraction = np.clip((points - start[:3, 3]) @ chord / max(float(chord @ chord), 1e-20), 0, 1)
    line_error = float(np.linalg.norm(points - start[:3, 3] - fraction[:, None] * chord, axis=1).max())
    if position_error > POSITION_TOLERANCE_M or angle_errors[-1] > ROTATION_TOLERANCE_RAD:
        raise ValueError("Endpoint pose mismatch")
    if linear and (line_error > POSITION_TOLERANCE_M or angle_errors.max() > ROTATION_TOLERANCE_RAD):
        raise ValueError("Straight/constant-orientation path check failed")
    return {"endpoint_position_error_m": position_error,
            "endpoint_rotation_error_rad": float(angle_errors[-1]),
            "line_error_m": line_error, "max_orientation_error_rad": float(angle_errors.max())}


def main() -> None:
    """读取请求并串联三个运动阶段；仅离线验证，不执行夹爪或 SDK 指令。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, default=Path(__file__).with_name("grasp_plan_request.json"))
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/smoke_test_trajectory.json")
    parser.add_argument("--plan-only", action="store_true", help="验证并导出后退出，不启动 Viser")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--autoplay", action="store_true", help="跳过终端 Enter，直接循环播放")
    parser.add_argument("--spheres", action="store_true", help="显示原生 cuRobo 碰撞球")
    args = parser.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be positive and finite")
    try:
        request, START, targets = load_request(args.request)
        from arx_r5.planning.collision import SphereChecker
        checker = SphereChecker()
        # 用户已确认 START；跳过单独的起点碰撞预检，继续尝试请求中的位姿规划。
        profile = load_profile()
        if any(q < lo or q > hi for q, lo, hi in zip(
                START, profile["position_lower"], profile["position_upper"])):
            raise ValueError("START outside offline model joint limits")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f"Request rejected: {error}\n")
    print("Request:", args.request.resolve(), "start:", START, flush=True)
    if not torch.cuda.is_available():
        parser.error("This example plans on GPU; run with the existing GPU-enabled .venv")

    # Step 1: 配置 Motion Planner，与 UR10e 一样直接创建规划器。
    content_path = ContentPath(
        robot_config_root_path=str(ROOT / "config"),
        robot_config_file="robot.yml",
        robot_urdf_absolute_path=str(ROOT / "models/r5_offline.urdf"),
        robot_asset_absolute_path=str(ROOT / "models"),
    )
    print("[1/7] Initialize MotionPlanner", flush=True)
    print("URDF:", content_path.robot_urdf_absolute_path, flush=True)
    print("Mesh root:", content_path.robot_asset_absolute_path, flush=True)
    config = MotionPlannerCfg.create(
        robot=robot_config(),
        scene_model=None,
        self_collision_check=True,
        num_ik_seeds=8,
        num_trajopt_seeds=2,
        use_cuda_graph=True,
        interpolation_dt=profile["interpolation_dt"],
        interpolation_buffer_size=500,
    )
    with MotionPlanner(config) as planner:
        print("Robot joints:", planner.joint_names, flush=True)
        print("Tool frames:", planner.tool_frames, flush=True)
        if planner.joint_names != JOINT_NAMES:
            raise ValueError("Unexpected R5 active joint order")
        if planner.tool_frames != ["link6"]:
            raise ValueError("Request requires model tool link6")

        # Step 2: 恒等映射；直接把请求位姿作为基座下的 link6 目标。
        print("[2/7] Set start and goal joint states", flush=True)
        q_start = JointState.from_position(
            torch.tensor([START], device="cuda", dtype=torch.float32),
            joint_names=planner.joint_names,
        )
        positions, velocities, accelerations, stages = [], [], [], []
        dt = float(planner.trajopt_solver.config.interpolation_dt)
        stage_start_transform = forward_kinematics(START)["link6"]
        for name, goal_transform in targets:
            print("[2/7] Stage:", name, "base_T_link6:", goal_transform, flush=True)
            quaternion = Rotation.from_matrix(goal_transform[:3, :3]).as_quat()[[3, 0, 1, 2]]
            goal_pose = GoalToolPose(
                tool_frames=planner.tool_frames,
                position=torch.tensor(goal_transform[:3, 3].reshape(1, 1, 1, 1, 3),
                                      device="cuda", dtype=torch.float32),
                quaternion=torch.tensor(quaternion.reshape(1, 1, 1, 1, 4),
                                        device="cuda", dtype=torch.float32),
            )
            # 采用官方 plan_grasp 的直线运动代价，同时约束横向位置及三个旋转轴。
            # grasp 使用目标工具坐标的 X，lift 使用基座坐标的 Z；不关闭碰撞。
            axis, project = (None, None)
            if name == "pregrasp":
                criteria = ToolPoseCriteria()
            else:
                axis, project = linear_axis(stage_start_transform, goal_transform)
                criteria = ToolPoseCriteria.linear_motion(
                    axis=axis, non_terminal_scale=1.0, project_distance_to_goal=project,
                )
            planner.update_tool_pose_criteria({"link6": criteria})
            # Step 3: 每阶段从上一阶段末点规划到 JSON 提供的末端位姿。
            print("[3/7] Planning with plan_pose", flush=True)
            result = planner.plan_pose(goal_pose, q_start)
            if result is None:
                raise RuntimeError(f"{name}: no IK solution found for the requested link6 pose; "
                                   "no trajectory exported")
            if result.success.numel() != 1 or not result.success.item():
                raise RuntimeError(f"{name}: trajectory optimization failed; no trajectory exported")

            # Step 4: 获取轨迹。当前版本返回 8 轴：先核对固定夹爪，再提取主动 6 轴。
            print("[4/7] Get interpolated trajectory", flush=True)
            trajectory = result.get_interpolated_plan()
            print("Raw trajectory shape:", trajectory.position.shape, flush=True)
            print("Raw trajectory joints:", trajectory.joint_names, flush=True)
            expected = JOINT_NAMES + list(profile["locked_gripper"])
            if (trajectory.joint_names is None or len(trajectory.joint_names) != len(expected)
                    or set(trajectory.joint_names) != set(expected)):
                raise ValueError("Unexpected interpolated joint names")
            for gripper_name, locked in profile["locked_gripper"].items():
                index = trajectory.joint_names.index(gripper_name)
                if not torch.allclose(trajectory.position[..., index],
                                      torch.full_like(trajectory.position[..., index], locked), atol=1e-6):
                    raise ValueError("Locked gripper position changed")
                if any(torch.any(getattr(trajectory, field)[..., index].abs() > 1e-6)
                       for field in ("velocity", "acceleration")):
                    raise ValueError("Locked gripper unexpectedly moving")
            trajectory = trajectory.reorder(planner.joint_names)
            joint_trajectory = trajectory.position[0, 0, :, :].detach()
            n_steps = joint_trajectory.shape[0]
            dt = float(planner.trajopt_solver.config.interpolation_dt)
            print("Playback shape:", joint_trajectory.shape, "frames:", n_steps, "dt:", dt, flush=True)

            stage_positions = joint_trajectory.cpu().tolist()
            # 与碰撞复核使用同一关节步长，加密检查输出点之间的线性关节插值。
            checked_positions = [stage_positions[0]]
            for a, b in zip(stage_positions[:-1], stage_positions[1:]):
                steps = max(1, int(np.ceil(max(abs(y - x) for x, y in zip(a, b))
                                          / profile["collision_max_joint_step"])))
                checked_positions.extend([[x + (y - x) * k / steps for x, y in zip(a, b)]
                                          for k in range(1, steps + 1)])
            stage_transforms = np.asarray([forward_kinematics(q)["link6"] for q in checked_positions],
                                          dtype=np.float32)
            try:
                pose_checks = validate_pose_path(stage_transforms, stage_start_transform,
                                                 goal_transform, linear=name != "pregrasp")
            except ValueError as error:
                raise ValueError(f"{name}: {error}") from error
            stage_data = make_trajectory(
                q_start.position[0].detach().cpu().tolist(), stage_positions[-1], stage_positions,
                trajectory.velocity[0, 0].detach().cpu().tolist(),
                trajectory.acceleration[0, 0].detach().cpu().tolist(), dt,
            )
            validate_numeric(stage_data)
            checker.validate(stage_positions)
            stages.append({"target": name, "first_sample": len(positions),
                           "last_sample": len(positions) + len(stage_positions) - 1,
                           "base_T_link6_goal": goal_transform.tolist(),
                           **pose_checks,
                           "linear_motion_axis": axis,
                           "linear_motion_frame": (None if project is None else
                                                   "goal_tool" if project else "base_link"),
                           "linear_path_checked": name != "pregrasp",
                           "pose_check_samples": len(checked_positions),
                           "solve_time_s": float(result.solve_time)})
            positions.extend(stage_positions)
            velocities.extend(stage_data["joint_velocities"])
            accelerations.extend(stage_data["joint_accelerations"])
            q_start = JointState.from_position(joint_trajectory[-1].unsqueeze(0),
                                               joint_names=planner.joint_names)
            stage_start_transform = goal_transform
        GOAL = positions[-1]
        joint_trajectory = torch.tensor(positions, device="cuda", dtype=torch.float32)
        n_steps = len(positions)

        # 保留原有数值、导数和独立自碰撞检查；失败时不进入可视化。
        data = make_trajectory(
            START, GOAL, joint_trajectory.cpu().tolist(),
            velocities, accelerations, dt,
        )
        data["planner"] = {"api": "MotionPlanner.plan_pose", "cuda_graph": True,
                           "version": version("nvidia-curobo"),
                           "solve_time_s": sum(stage["solve_time_s"] for stage in stages),
                           "optimized_dt_s": result.js_solution.dt.detach().cpu().tolist()}
        # 用实际轨迹 FK 量化“平移/旋转”，不从六轴角度或截图主观判断。
        tool_transforms = np.array([forward_kinematics(q)["link6"]
                                    for q in joint_trajectory.cpu().tolist()], dtype=np.float32)
        tool_points = tool_transforms[:, :3, 3]
        rotations = tool_transforms[:, :3, :3]
        rotation_drift = Rotation.from_matrix(rotations[0].T @ rotations).magnitude()
        chord = tool_points[-1] - tool_points[0]
        length = float(np.linalg.norm(chord))
        fraction = np.clip((tool_points - tool_points[0]) @ chord / max(length ** 2, 1e-20), 0.0, 1.0)
        line_error = np.linalg.norm(tool_points - tool_points[0] - fraction[:, None] * chord, axis=1)
        data["task_space_diagnostics"] = {
            "reference_frame": "link6 (URDF wrist, not calibrated gripper TCP)",
            "start_base_T_link6": tool_transforms[0].tolist(),
            "goal_base_T_link6": tool_transforms[-1].tolist(),
            "endpoint_displacement_m": length,
            "path_length_m": float(np.linalg.norm(np.diff(tool_points, axis=0), axis=1).sum()),
            "max_orientation_drift_deg": float(np.degrees(rotation_drift).max()),
            "max_deviation_from_endpoint_line_m": float(line_error.max()),
            "cartesian_orientation_constraint_enforced": False,
            "cartesian_straight_line_constraint_enforced": False,
            "linear_constant_orientation_stages": ["grasp", "lift"],
        }
        print("Actual link6 translation [m]:", length, "max rotation [deg]:",
              data["task_space_diagnostics"]["max_orientation_drift_deg"], flush=True)
        data["grasp_request"] = {"path": str(args.request.resolve()),
                                 "sha256": content_digest(request),
                                 "coordinate_mapping": "identity_user_confirmed",
                                 "tool_offset_reapplied": False,
                                 "pose_check_method": "densified_joint_linear_interpolation",
                                 "pose_check_max_joint_step_rad": profile["collision_max_joint_step"],
                                 "continuous_cartesian_path_proof": False,
                                 "stages": stages, "scene_collision_checked": False,
                                 "gripper_actions_executed": False,
                                 "gripper_actions_requested": [stage for stage in request["task"]["stages"]
                                                               if stage["action"] == "set_catch_pos"],
                                 "path_check_tolerances": {"position_m": POSITION_TOLERANCE_M,
                                                           "rotation_rad": ROTATION_TOLERANCE_RAD}}
        # 用户确认恒等映射后，SDK 六轴列与模型六轴列完全相同。
        for field in ("joint_positions", "joint_velocities", "joint_accelerations"):
            data[field + "_sdk"] = [row.copy() for row in data[field]]
        export_trajectory(args.output, data)
        print("Trajectory checks passed; exported", args.output.resolve(), flush=True)
        if args.plan_only:
            return

        # Step 5: 与 UR10e 完全相同的可视化类，显式初始化并加载 URDF 网格。
        print("[5/7] Initialize curobo.viewer.ViserVisualizer", flush=True)
        viewer = ViserVisualizer(
            content_path=content_path,
            connect_ip=args.host,
            connect_port=args.port,
            add_robot_to_scene=True,
            add_control_frames=True,
            visualize_robot_spheres=args.spheres,
        )
        # 当前 cuRobo 没有公开 server 属性；这里只用它设置相机、辅助路径及状态文字。
        # 机器人加载与运动全部由原生 ViserVisualizer / set_joint_state 完成。
        server = viewer._server
        try:
            print("Loaded visual meshes:", len(viewer._urdf.scene.geometry), flush=True)
            print("Viewer joints:", viewer.joint_names, flush=True)
            if not viewer._urdf.scene.geometry:
                raise RuntimeError("URDF loaded no visual meshes")
            server.scene.set_up_direction("+z")
            server.initial_camera.position = (1.0, -0.7, 0.8)
            server.initial_camera.look_at = (0.07, 0.0, 0.28)
            server.initial_camera.up = (0.0, 0.0, 1.0)
            server.initial_camera.fov = np.deg2rad(45.0)
            server.scene.add_grid("/ground_plane", width=1.4, height=1.4, cell_size=0.05,
                                  section_size=0.25, position=(0.0, 0.0, -0.002))
            server.scene.add_line_segments(
                "/tool_path", points=np.stack((tool_points[:-1], tool_points[1:]), axis=1),
                colors=(45, 120, 225), thickness=0.004,
            )
            status = server.gui.add_markdown("**R5 / native ViserVisualizer / offline only**")

            # Step 6: 先显示起点，然后等浏览器打开、终端 Enter；与 UR10e 示例一致。
            print("[6/7] Set initial state", flush=True)
            initial_state = JointState.from_position(joint_trajectory[0], joint_names=planner.joint_names)
            viewer.set_joint_state(initial_state)
            print(f"Open: http://{args.host}:{server.get_port()}", flush=True)
            if server.get_port() != args.port:
                print(f"Requested port {args.port} was busy; use the actual URL above.", flush=True)
            if not args.autoplay:
                try:
                    input("Open the viewer, then press Enter to play... ")
                except EOFError as error:
                    raise RuntimeError("Run in an interactive terminal or pass --autoplay") from error

            # Step 7: 与 UR10e 一样逐帧构造 JointState，调用 set_joint_state 并 sleep。
            print("[7/7] Playback started; Ctrl+C stops this offline viewer", flush=True)
            while True:
                for i in range(n_steps):
                    joint_position = joint_trajectory[i]
                    joint_state = JointState.from_position(
                        joint_position, joint_names=planner.joint_names,
                    )
                    viewer.set_joint_state(joint_state)
                    status.content = (f"**R5 / native ViserVisualizer / offline only**\n\n"
                                      f"Frame {i + 1}/{n_steps} · t = {i * dt:.3f} s")
                    time.sleep(dt / args.speed)
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nVisualization stopped.", flush=True)
        finally:
            server.stop()


if __name__ == "__main__":
    main()
