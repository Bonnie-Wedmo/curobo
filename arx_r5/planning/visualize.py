"""导出关节曲线/模型图，或用 Viser 查看网格、碰撞球与轨迹回放。"""
from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from arx_r5.common import ROOT, load_profile
from arx_r5.planning.model import forward_kinematics, link_meshes, robot_config
from arx_r5.trajectory import load_trajectory


def plot_trajectory(data: dict, output: Path) -> None:
    """保存全部六关节位置、速度、加速度曲线，单位显式标注。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, key, unit in zip(axes, ["joint_positions", "joint_velocities", "joint_accelerations"],
                             ["Position [rad]", "Velocity [rad/s]", "Acceleration [rad/s²]"]):
        for i, name in enumerate(data["joint_names"]):
            ax.plot(data["time_from_start"], [row[i] for row in data[key]], label=name)
        ax.set_ylabel(unit)
        ax.grid(alpha=0.3)
    axes[0].legend(ncol=6)
    axes[0].set_title("ARX R5 — offline cuRobo joint trajectory (NOT hardware execution)")
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_model(q: list[float], output: Path) -> None:
    """两幅相同坐标视图展示原网格与碰撞球，可导出 PNG。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import trimesh

    fk = forward_kinematics(q)
    meshes = link_meshes()
    spheres = robot_config()["robot_cfg"]["kinematics"]["collision_spheres"]
    fig = plt.figure(figsize=(14, 7))
    all_vertices = []
    unit = trimesh.creation.icosphere(subdivisions=1)
    for plot_index in range(2):
        ax = fig.add_subplot(1, 2, plot_index + 1, projection="3d")
        for i, (name, mesh) in enumerate(meshes.items()):
            color = plt.get_cmap("tab10")(i)
            vertices = mesh.vertices @ fk[name][:3, :3].T + fk[name][:3, 3]
            all_vertices.extend(vertices)
            if plot_index == 0:
                ax.add_collection3d(Poly3DCollection(vertices[mesh.faces], facecolor=color,
                                                     alpha=0.85, linewidth=0))
            else:
                for s in spheres[name]:
                    center = fk[name][:3, :3] @ s["center"] + fk[name][:3, 3]
                    v = unit.vertices * s["radius"] + center
                    ax.add_collection3d(Poly3DCollection(v[unit.faces], facecolor=color,
                                                         alpha=0.22, linewidth=0))
        points = np.asarray(all_vertices)
        center = (points.min(0) + points.max(0)) * 0.5
        half = float((points.max(0) - points.min(0)).max()) * 0.6
        ax.set(xlim=(center[0] - half, center[0] + half),
               ylim=(center[1] - half, center[1] + half),
               zlim=(center[2] - half, center[2] + half), xlabel="X [m]", ylabel="Y [m]", zlabel="Z [m]")
        ax.set_box_aspect((1, 1, 1))
        ax.set_title("Official R5a meshes" if plot_index == 0 else "Conservative collision spheres")
    fig.suptitle("Offline model; fixed-open gripper; hardware geometry NOT commissioned")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def prepare_scene(data: dict | None) -> dict[str, Any]:
    """预计算 FK、末端路径及整个运动的网格边界，避免默认相机远离半米级机械臂。"""
    positions = data["joint_positions"] if data else [load_profile()["default_position"]]
    meshes = link_meshes()
    transforms = [forward_kinematics(q) for q in positions]
    corners = []
    for fk in transforms:
        for name, mesh in meshes.items():
            corners.append(mesh.bounding_box.vertices @ fk[name][:3, :3].T + fk[name][:3, 3])
    points = np.concatenate(corners)
    bounds = np.array([points.min(0), points.max(0)])
    center = bounds.mean(0)
    radius = max(float(np.linalg.norm(bounds[1] - bounds[0])) * 0.5, 0.15)
    direction = np.array([1.4, -0.25, 0.85])
    camera = center + direction / np.linalg.norm(direction) * radius * 3.0
    return {"meshes": meshes, "transforms": transforms, "bounds": bounds,
            "tool_positions": np.array([fk["link6"][:3, 3] for fk in transforms]),
            "center": center, "camera": camera}


def view(data: dict | None, port: int, smoke_seconds: float | None = None,
         host: str = "127.0.0.1", autoplay: bool = True) -> None:
    """显示完整运动范围、起终点轮廓及真实采样末端路径；使用 HTTP/WebSocket。"""
    import viser
    import trimesh

    prepared = prepare_scene(data)
    positions = data["joint_positions"] if data else [load_profile()["default_position"]]
    transforms, tool = prepared["transforms"], prepared["tool_positions"]
    times = np.array(data["time_from_start"] if data else [0.0])
    server = viser.ViserServer(host=host, port=port, label="ARX R5 trajectory")
    server.gui.configure_theme(control_layout="floating", control_width="medium",
                               show_logo=False, show_share_button=False)
    server.scene.set_up_direction("+z")
    server.initial_camera.position = prepared["camera"]
    server.initial_camera.look_at = prepared["center"]
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.initial_camera.fov = np.deg2rad(45.0)
    server.initial_camera.near = 0.005
    server.scene.add_grid("/ground", width=1.6, height=1.6, cell_size=0.05,
                          section_size=0.25, position=(0.0, 0.0, -0.002),
                          cell_color=(210, 219, 228), section_color=(160, 174, 190))
    server.scene.add_frame("/base_axes", axes_length=0.08, axes_radius=0.002)
    displacement = float(np.linalg.norm(tool[-1] - tool[0]))
    server.gui.add_markdown(
        f"## ARX R5 · trajectory replay\n"
        f"**Offline model / no hardware commands**\n\n"
        f"{len(positions)} waypoints · {times[-1]:.2f} s · "
        f"end-to-end link6 displacement **{displacement * 100:.1f} cm**\n\n"
        "Green = start · Orange = goal · Blue = sampled link6 path")
    spheres = robot_config()["robot_cfg"]["kinematics"]["collision_spheres"]
    frames, sphere_handles = {}, []
    unit = trimesh.creation.icosphere(subdivisions=1)
    ghosts = []
    for i, (name, mesh) in enumerate(prepared["meshes"].items()):
        initial = transforms[0][name]
        xyzw = Rotation.from_matrix(initial[:3, :3]).as_quat()
        frames[name] = server.scene.add_frame(f"/robot/{name}", show_axes=False,
                                               position=initial[:3, 3], wxyz=xyzw[[3, 0, 1, 2]])
        server.scene.add_mesh_simple(f"/robot/{name}/mesh", mesh.vertices.astype(np.float32),
                                     mesh.faces.astype(np.uint32), side="double", material="toon3",
                                     color=(35, 90, 155) if i % 2 else (125, 145, 170))
        if data:
            for label, fk, color in [("start", transforms[0], (44, 166, 99)),
                                      ("goal", transforms[-1], (234, 142, 43))]:
                xyzw = Rotation.from_matrix(fk[name][:3, :3]).as_quat()
                ghosts.append(server.scene.add_mesh_simple(
                    f"/reference/{label}/{name}", mesh.vertices.astype(np.float32),
                    mesh.faces.astype(np.uint32), color=color, opacity=0.12, side="double",
                    position=fk[name][:3, 3], wxyz=xyzw[[3, 0, 1, 2]], cast_shadow=False))
        items = spheres[name]
        sphere_handles.append(server.scene.add_batched_meshes_simple(
            f"/robot/{name}/spheres", unit.vertices.astype(np.float32), unit.faces.astype(np.uint32),
            batched_wxyzs=np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (len(items), 1)),
            batched_positions=np.array([s["center"] for s in items], dtype=np.float32),
            batched_scales=np.array([s["radius"] for s in items], dtype=np.float32),
            batched_colors=(250, 140, 45), opacity=0.25, wireframe=True, visible=False))
    path_handles = []
    if len(tool) > 1:
        # 相邻 FK 点连线，绝不以平滑曲线代替实际采样路径。
        path_handles.append(server.scene.add_line_segments(
            "/trajectory/path", np.stack((tool[:-1], tool[1:]), axis=1).astype(np.float32),
            colors=(48, 127, 224), thickness=0.004))
        for label, point, color in [("START", tool[0], (44, 166, 99)),
                                     ("GOAL", tool[-1], (234, 142, 43))]:
            path_handles.append(server.scene.add_icosphere(
                f"/trajectory/{label}", radius=0.009, color=color, position=point))
            path_handles.append(server.scene.add_label(
                f"/labels/{label}", label, position=point + np.array([0.025, 0.0, 0.025])))
    tip = server.scene.add_frame("/tool", axes_length=0.06, axes_radius=0.0025)
    play = server.gui.add_checkbox("Play", autoplay and len(positions) > 1)
    repeat = server.gui.add_checkbox("Loop", True)
    speed = server.gui.add_slider("Speed", 0.1, 2.0, 0.1, 0.6)
    slider = server.gui.add_slider("Waypoint", 0, max(1, len(positions) - 1), 1, 0)
    visible = server.gui.add_checkbox("Collision spheres", False)
    reference = server.gui.add_checkbox("Start / goal ghosts", True)
    path_visible = server.gui.add_checkbox("Tool path", True)
    status = server.gui.add_markdown("")
    reset_camera = server.gui.add_button("Reset camera")
    to_start = server.gui.add_button("Go to start")
    to_goal = server.gui.add_button("Go to goal")

    @reset_camera.on_click
    def reset_view(_: Any) -> None:
        for client in server.get_clients().values():
            client.camera.position = prepared["camera"]
            client.camera.look_at = prepared["center"]
            client.camera.up_direction = (0.0, 0.0, 1.0)

    @to_start.on_click
    def show_start(_: Any) -> None:
        play.value, slider.value = False, 0

    @to_goal.on_click
    def show_goal(_: Any) -> None:
        play.value, slider.value = False, len(positions) - 1

    started = time.monotonic()
    previous_time, cursor, shown = started, 0.0, -1
    previous_visibility = None
    print(f"Open HTTP viewer: http://{host}:{server.get_port()} (keep this process running)", flush=True)
    try:
        while smoke_seconds is None or time.monotonic() - started < smoke_seconds:
            now = time.monotonic()
            elapsed, previous_time = now - previous_time, now
            if slider.value != shown and shown >= 0:
                cursor = float(times[min(slider.value, len(positions) - 1)])
            if play.value and len(positions) > 1:
                cursor += elapsed * speed.value
                if cursor > times[-1] + speed.value:
                    if repeat.value:
                        cursor = 0.0
                    else:
                        play.value = False
                slider.value = min(int(np.searchsorted(times, cursor, side="right") - 1), len(positions) - 1)
            index = min(slider.value, len(positions) - 1)
            if index != shown:
                with server.atomic():
                    for name, transform in transforms[index].items():
                        frames[name].position = transform[:3, 3]
                        xyzw = Rotation.from_matrix(transform[:3, :3]).as_quat()
                        frames[name].wxyz = xyzw[[3, 0, 1, 2]]
                    transform = transforms[index]["link6"]
                    tip.position = transform[:3, 3]
                    tip.wxyz = Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]
                    status.content = (f"**t = {times[index]:.2f} / {times[-1]:.2f} s** · "
                                      f"sample {index + 1}/{len(positions)}\n\n"
                                      f"q [rad]: {', '.join(f'{v:.3f}' for v in positions[index])}")
                shown = index
            visibility = (visible.value, reference.value, path_visible.value)
            if visibility != previous_visibility:
                for handles, flag in zip((sphere_handles, ghosts, path_handles), visibility):
                    for handle in handles:
                        handle.visible = flag
                previous_visibility = visibility
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


def main() -> None:
    """默认导出文件；显式 --viewer 才启动本机浏览器服务。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--paused", action="store_true", help="启动时暂停，便于逐点检查")
    parser.add_argument("--smoke-seconds", type=float)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts")
    args = parser.parse_args()
    data = load_trajectory(args.trajectory) if args.trajectory else None
    if args.viewer:
        view(data, args.port, args.smoke_seconds, args.host, not args.paused)
    else:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        if data:
            plot_trajectory(data, args.output_dir / "joint_trajectory.png")
        plot_model(data["start_state"] if data else load_profile()["default_position"],
                   args.output_dir / "model_spheres.png")
        print("Plots:", args.output_dir)


if __name__ == "__main__":
    main()
