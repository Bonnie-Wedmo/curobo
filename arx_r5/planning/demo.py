"""按 UR10e 学习示例的顺序：规划、提取轨迹、初始化 Viser、逐帧回放。"""
from __future__ import annotations

import argparse
import time
from importlib.metadata import version

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import ContentPath, JointState
from curobo.viewer import ViserVisualizer

from arx_r5.common import JOINT_NAMES, ROOT, load_profile
from arx_r5.planning.model import forward_kinematics, robot_config
from arx_r5.trajectory import export_trajectory, make_trajectory

START = [0.0, 0.0, 1.1, -1.0, 0.13, 0.06]
GOAL = [0.2, 0.02, 0.45, -0.35, 0.11, 0.04]

def main() -> None:
    """运行与 UR10e 同款的原生 ViserVisualizer；默认等待终端 Enter 后播放。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--autoplay", action="store_true", help="跳过终端 Enter，直接循环播放")
    parser.add_argument("--spheres", action="store_true", help="显示原生 cuRobo 碰撞球")
    parser.add_argument("--move-to-start", action="store_true",
                        help="请求真机移到 START；SDK 未完成现场验证时拒绝执行")
    args = parser.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be positive and finite")
    # Step 0: 真机准备的独立入口。当前 SDK 尚未验收，在连接设备前拒绝执行。
    # 后续应读取实际状态，规划并验证 current → START，执行并反馈确认到位。
    if args.move_to_start:
        from arx_r5.control.sdk import OfficialR5Adapter
        try:
            OfficialR5Adapter().move_to_start(START)
        except (ValueError, RuntimeError) as error:
            parser.exit(2, f"Move-to-START rejected: {error}\n")
    if not torch.cuda.is_available():
        parser.error("This example plans on GPU; run with the existing GPU-enabled .venv")

    # Step 1: 配置 Motion Planner，与 UR10e 一样直接创建规划器。
    profile = load_profile()
    content_path = ContentPath(
        robot_config_root_path=str(ROOT / "config"),
        robot_config_file="robot.yml",
        robot_urdf_absolute_path=str(ROOT / "models/r5_offline.urdf"),
        robot_asset_absolute_path=str(ROOT / "models"),
    )
    print("[1/7] Initialize MotionPlanner", flush=True)
    print("URDF:", content_path.robot_urdf_absolute_path, flush=True)
    print("Mesh root:", content_path.robot_asset_absolute_path, flush=True)
    from arx_r5.planning.collision import SphereChecker
    SphereChecker().validate([START])
    SphereChecker().validate([GOAL])
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

        # Step 2: 设置六轴起点和目标；本项目采用关节目标，所以调用 plan_cspace。
        print("[2/7] Set start and goal joint states", flush=True)
        q_start = JointState.from_position(
            torch.tensor([START], device="cuda", dtype=torch.float32),
            joint_names=planner.joint_names,
        )
        q_goal = JointState.from_position(
            torch.tensor([GOAL], device="cuda", dtype=torch.float32),
            joint_names=planner.joint_names,
        )
        print("Start:", q_start.position, flush=True)
        print("Goal:", q_goal.position, flush=True)

        # Step 3: 实际调用 cuRobo 规划，不再默认跳到已有 JSON 的回放流程。
        print("[3/7] Planning with plan_cspace", flush=True)
        result = planner.plan_cspace(goal_state=q_goal, current_state=q_start)
        if result is None or result.success.numel() != 1 or not result.success.item():
            raise RuntimeError("Planning failed; viewer was not initialized")

        # Step 4: 获取轨迹。当前版本返回 8 轴：先核对固定夹爪，再提取主动 6 轴。
        print("[4/7] Get interpolated trajectory", flush=True)
        trajectory = result.get_interpolated_plan()
        print("Raw trajectory shape:", trajectory.position.shape, flush=True)
        print("Raw trajectory joints:", trajectory.joint_names, flush=True)
        expected = JOINT_NAMES + list(profile["locked_gripper"])
        if (trajectory.joint_names is None or len(trajectory.joint_names) != len(expected)
                or set(trajectory.joint_names) != set(expected)):
            raise ValueError("Unexpected interpolated joint names")
        for name, locked in profile["locked_gripper"].items():
            index = trajectory.joint_names.index(name)
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

        # 保留原有数值、导数和独立自碰撞检查；失败时不进入可视化。
        data = make_trajectory(
            START, GOAL, joint_trajectory.cpu().tolist(),
            trajectory.velocity[0, 0].detach().cpu().tolist(),
            trajectory.acceleration[0, 0].detach().cpu().tolist(), dt,
        )
        data["planner"] = {"api": "MotionPlanner.plan_cspace", "cuda_graph": True,
                           "version": version("nvidia-curobo"),
                           "solve_time_s": float(result.solve_time),
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
        }
        print("Actual link6 translation [m]:", length, "max rotation [deg]:",
              data["task_space_diagnostics"]["max_orientation_drift_deg"], flush=True)
        export_trajectory(ROOT / "artifacts/visual_demo_trajectory.json", data)
        print("Trajectory checks passed; exported visual_demo_trajectory.json", flush=True)

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
