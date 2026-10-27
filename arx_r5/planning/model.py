"""URDF 独立 FK、网格包络球及模型对比；不依赖 CUDA。"""
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
import yaml
from scipy.spatial.transform import Rotation

from arx_r5.common import JOINT_NAMES, ROOT, digest, load_profile, write_json


def origin_matrix(element: ET.Element | None) -> np.ndarray:
    """URDF origin: 平移后 Rz(yaw) Ry(pitch) Rx(roll)，单位 m/rad。"""
    result = np.eye(4, dtype=np.float32)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        result[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return result


def forward_kinematics(q: list[float], path: Path | None = None) -> dict[str, np.ndarray]:
    """按 URDF 树计算所有 link 的 base_T_link，六个角度按 joint1..6 排序。"""
    path = path or ROOT / "models/r5_offline.urdf"
    positions = dict(zip(JOINT_NAMES, q, strict=True))
    positions.update(load_profile()["locked_gripper"])
    result = {"base_link": np.eye(4, dtype=np.float32)}
    pending = list(ET.parse(path).getroot().findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in result:
                continue
            transform = origin_matrix(joint.find("origin"))
            motion = np.eye(4, dtype=np.float32)
            if joint.get("type") != "fixed":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ", dtype=np.float32)
                value = positions[joint.get("name")]
                if joint.get("type") == "prismatic":
                    motion[:3, 3] = axis * value
                else:
                    motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
            result[child] = result[parent] @ transform @ motion
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("URDF has disconnected links or a cycle")
    return result


def link_meshes() -> dict[str, trimesh.Trimesh]:
    """把 collision mesh 的 scale/origin 应用于顶点，返回 link 局部坐标网格。"""
    result = {}
    for link in ET.parse(ROOT / "models/r5_offline.urdf").getroot().findall("link"):
        meshes = []
        for collision in link.findall("collision"):
            mesh_element = collision.find("geometry/mesh")
            mesh = trimesh.load_mesh(ROOT / "models" / mesh_element.get("filename"))
            mesh.apply_scale(np.fromstring(mesh_element.get("scale", "1 1 1"), sep=" "))
            mesh.apply_transform(origin_matrix(collision.find("origin")))
            meshes.append(mesh)
        result[link.get("name")] = trimesh.util.concatenate(meshes)
    return result


def clip_polygon(polygon: list[np.ndarray], boundary: float, keep_above: bool,
                 axis: int = 0) -> list[np.ndarray]:
    """在主轴坐标 x 上裁剪三角形；用于包住整片三角形而非仅采样顶点。"""
    output = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        inside_a = a[axis] >= boundary if keep_above else a[axis] <= boundary
        inside_b = b[axis] >= boundary if keep_above else b[axis] <= boundary
        if inside_a:
            output.append(a)
        if inside_a != inside_b:
            output.append(a + (b - a) * ((boundary - a[axis]) / (b[axis] - a[axis])))
    return output


def covering_spheres(mesh: trimesh.Trimesh, width: float, padding: float) -> list[dict]:
    """递归裁剪三角面片，包络球最大直径为 width；球心在 link 局部坐标。"""
    vertices = np.asarray(mesh.vertices, dtype=np.float32)
    _, axes = np.linalg.eigh(np.cov(vertices.T))
    axes = axes[:, ::-1]
    points = vertices @ axes
    triangles = points[mesh.faces]
    spheres = []

    def partition(polygons: list[list[np.ndarray]]) -> None:
        flat = np.concatenate(polygons)
        lo, hi = flat.min(0), flat.max(0)
        center = (lo + hi) * 0.5
        radius = np.linalg.norm(flat - center, axis=1).max()
        if radius <= width * 0.5:
            spheres.append({"center": (center @ axes.T).tolist(),
                            "radius": float(radius + padding)})
            return
        axis = int(np.argmax(hi - lo))
        split = float(center[axis])
        for above in (False, True):
            clipped = [clip_polygon(poly, split, above, axis) for poly in polygons]
            nonempty = [poly for poly in clipped if len(poly) >= 3]
            if nonempty:
                partition(nonempty)

    partition([list(triangle) for triangle in triangles])
    return spheres


def robot_config() -> dict[str, Any]:
    """YAML 保持可搬迁；加载时仅将模型路径解析到本模块绝对路径。"""
    cfg = yaml.safe_load((ROOT / "config/robot.yml").read_text())
    kin = cfg["robot_cfg"]["kinematics"]
    kin["urdf_path"] = str(ROOT / "models/r5_offline.urdf")
    kin["asset_root_path"] = str(ROOT / "models")
    return cfg


def add_elbow_body_guards(kin: dict[str, Any], profile: dict[str, Any]) -> None:
    """给肘部两侧远离转轴壳体的球建立固定别名，让 GPU 和 CPU 都检查躯体干涉。"""
    guard = profile["adjacent_body_guard"]
    joint = ET.parse(ROOT / "models/r5_offline.urdf").getroot().find(
        f"joint[@name='{guard['joint_name']}']")
    if (joint is None or joint.find("parent").get("link") != guard["parent_link"]
            or joint.find("child").get("link") != guard["child_link"]):
        raise ValueError("Elbow guard joint does not match URDF")
    physical_links = kin["collision_link_names"].copy()
    pivots = [origin_matrix(joint.find("origin"))[:3, 3], np.zeros(3, dtype=np.float32)]
    aliases = []
    for link, pivot in zip((guard["parent_link"], guard["child_link"]), pivots):
        alias = f"{link}_elbow_body"
        # 只保留完全位于关节壳体邻域之外的原球，半径和缓冲均不缩小。
        body = [dict(s) for s in kin["collision_spheres"][link]
                if np.linalg.norm(np.asarray(s["center"]) - pivot) - s["radius"] >
                guard["excluded_joint_radius_m"]]
        if not body:
            raise ValueError("Elbow body guard is empty")
        kin["collision_spheres"][alias] = body
        kin["collision_link_names"].append(alias)
        kin["self_collision_buffer"][alias] = 0.0
        kin.setdefault("extra_links", {})[alias] = {
            "link_name": alias, "parent_link_name": link, "joint_name": f"{alias}_fixed",
            "joint_type": "FIXED", "fixed_transform": [0, 0, 0, 1, 0, 0, 0],
            "link_mass": 0.0, "link_inertia": [0.0] * 6,
        }
        # 原始非相邻球对照常检查；这两个别名只额外相互检查，不重复检查其他球对。
        kin["self_collision_ignore"][alias] = physical_links.copy()
        aliases.append(alias)
    if aliases[1] in kin["self_collision_ignore"][aliases[0]]:
        raise ValueError("Elbow body guard pair must not be excluded")


def build() -> None:
    """生成独立 URDF 和 YAML；不修改任何上游原件或现有机器人配置。"""
    profile = load_profile()
    tree = ET.parse(ROOT / "models/upstream/R5a.urdf")
    for mesh in tree.getroot().iter("mesh"):
        mesh.set("filename", "meshes/" + mesh.get("filename").split("/")[-1])
    for joint in tree.getroot().findall("joint"):
        name = joint.get("name")
        if name in JOINT_NAMES:
            i = JOINT_NAMES.index(name)
            limit = joint.find("limit")
            limit.set("lower", str(profile["position_lower"][i]))
            limit.set("upper", str(profile["position_upper"][i]))
            limit.set("velocity", str(profile["velocity"][i]))
    tree.write(ROOT / "models/r5_offline.urdf", encoding="utf-8", xml_declaration=True)
    spheres = {name: covering_spheres(mesh, profile["sphere_link_diameter_m"].get(
                                          name, profile["sphere_max_diameter_m"]),
                                      profile["collision_padding_m"])
               for name, mesh in link_meshes().items()}
    # 关节连接区域仍用相邻排除；随后给肘部躯体添加独立球组，避免整对排除漏检。
    ignores: dict[str, list[str]] = {}
    for joint in tree.getroot().findall("joint"):
        ignores.setdefault(joint.find("parent").get("link"), []).append(
            joint.find("child").get("link"))
    cfg = {"robot_cfg": {"kinematics": {
        "urdf_path": "models/r5_offline.urdf", "asset_root_path": "models",
        "base_link": "base_link", "tool_frames": ["link6"],
        "lock_joints": profile["locked_gripper"],
        "collision_link_names": list(spheres), "mesh_link_names": list(spheres),
        "collision_spheres": spheres, "collision_sphere_buffer": 0.0,
        "self_collision_ignore": ignores,
        "self_collision_buffer": {name: 0.0 for name in spheres},
        "cspace": {"joint_names": JOINT_NAMES,
                   "default_joint_position": profile["default_position"],
                   "cspace_distance_weight": [1.0] * 6, "null_space_weight": [1.0] * 6,
                   "max_acceleration": profile["acceleration"], "max_jerk": profile["jerk"],
                   "position_limit_clip": 0.0}}}}
    add_elbow_body_guards(cfg["robot_cfg"]["kinematics"], profile)
    (ROOT / "config/robot.yml").write_text(
        "# OFFLINE ONLY: velocity/acceleration and hardware model are NOT commissioned.\n"
        + yaml.safe_dump(cfg, sort_keys=False))


def comparison() -> dict[str, Any]:
    """逐关节对比两份官方 URDF，不强行合并坐标定义。"""
    paths = [ROOT / "models/upstream/R5a.urdf", ROOT / "models/upstream/R5_master.urdf"]
    roots = [ET.parse(p).getroot() for p in paths]
    differences = []
    for a in roots[0].findall("joint"):
        b = roots[1].find(f"joint[@name='{a.get('name')}']")
        if b is None:
            differences.append({"joint": a.get("name"), "SDK_model": "missing"})
            continue
        for tag in ["parent", "child", "origin", "axis", "limit"]:
            if a.find(tag).attrib != b.find(tag).attrib:
                differences.append({"joint": a.get("name"), "field": tag,
                                    "R5a": a.find(tag).attrib, "SDK": b.find(tag).attrib})
    link_differences = []
    for a in roots[0].findall("link"):
        b = roots[1].find(f"link[@name='{a.get('name')}']")
        if b is None:
            link_differences.append({"link": a.get("name"), "SDK_model": "missing"})
            continue
        for tag in ["inertial/origin", "inertial/mass", "inertial/inertia",
                    "visual/origin", "visual/geometry/mesh", "collision/origin",
                    "collision/geometry/mesh"]:
            aa, bb = a.find(tag), b.find(tag)
            av, bv = aa.attrib if aa is not None else None, bb.attrib if bb is not None else None
            if av != bv:
                link_differences.append({"link": a.get("name"), "field": tag,
                                         "R5a": av, "SDK": bv})
    states = [[0.0] * 6, load_profile()["default_position"], [0.2, -1.3, 0.5, 0.2, -0.4, 0.3]]
    fk = []
    for q in states:
        a, b = [forward_kinematics(q, path)["link6"] for path in paths]
        fk.append({"q_rad": q, "R5a_base_T_link6": a.tolist(), "SDK_base_T_link6": b.tolist(),
                   "position_difference_m": float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
                   "rotation_difference_rad": float(Rotation.from_matrix(
                       a[:3, :3].T @ b[:3, :3]).magnitude())})
    return {"source_sha256": {p.name: digest(p) for p in paths},
            "joint_differences": differences, "link_differences": link_differences,
            "fk_comparison": fk,
            "conclusion": "Not equivalent. Revision/end-effector cause is not documented; hardware blocked."}


def main() -> None:
    """重建模型并输出可复查的比较结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    args = parser.parse_args()
    if args.build:
        build()
    report = comparison()
    write_json(ROOT / "artifacts/model_comparison.json", report)
    print("Model comparison:", ROOT / "artifacts/model_comparison.json")
    print("Joint names:", JOINT_NAMES)
    print("Locked gripper:", load_profile()["locked_gripper"])
    print("Hardware eligible: False")


if __name__ == "__main__":
    main()
