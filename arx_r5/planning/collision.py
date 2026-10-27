"""用独立 CPU FK 复查碰撞球；不信任轨迹文件中自报的通过标志。"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from arx_r5.common import load_profile
from arx_r5.planning.model import forward_kinematics, robot_config


class SphereChecker:
    """检查配置中的球对，包括相邻肘部两侧躯体的固定别名球组。"""

    def __init__(self) -> None:
        cfg = robot_config()["robot_cfg"]["kinematics"]
        self.spheres = cfg["collision_spheres"]
        self.links = list(self.spheres)
        self.extra_links = cfg.get("extra_links", {})
        for extra in self.extra_links.values():
            if extra["fixed_transform"] != [0, 0, 0, 1, 0, 0, 0]:
                raise ValueError("CPU checker only supports identity body-guard aliases")
        ignored = {frozenset((a, b)) for a, others in cfg["self_collision_ignore"].items()
                   for b in others}
        self.pairs = [(a, b) for i, a in enumerate(self.links) for b in self.links[i + 1:]
                      if frozenset((a, b)) not in ignored]
        self.local = {name: np.asarray([s["center"] for s in items], dtype=np.float32)
                      for name, items in self.spheres.items()}
        self.radii = {name: np.asarray([s["radius"] for s in items], dtype=np.float32)
                      for name, items in self.spheres.items()}

    def clearance(self, q: list[float]) -> tuple[float, tuple[str, str]]:
        """返回最小表面间距 m 和对应 link 对，负数表示穿透。"""
        fk = forward_kinematics(q)
        for name, extra in self.extra_links.items():
            fk[name] = fk[extra["parent_link_name"]]
        world = {name: self.local[name] @ fk[name][:3, :3].T + fk[name][:3, 3]
                 for name in self.links}
        minimum, pair = float("inf"), ("", "")
        for a, b in self.pairs:
            distance = np.linalg.norm(world[a][:, None] - world[b][None, :], axis=-1)
            gap = float((distance - self.radii[a][:, None] - self.radii[b][None, :]).min())
            if gap < minimum:
                minimum, pair = gap, (a, b)
        return minimum, pair

    def validate(self, positions: list[list[float]]) -> dict[str, Any]:
        """加密检查线性关节段；这不是任意控制器插值的连续碰撞证明。"""
        max_step = load_profile()["collision_max_joint_step"]
        minimum, count, closest_pair = float("inf"), 0, None
        samples = [positions[0]]
        for a, b in zip(positions[:-1], positions[1:]):
            n = max(1, math.ceil(max(abs(y - x) for x, y in zip(a, b)) / max_step))
            samples.extend([[x + (y - x) * k / n for x, y in zip(a, b)]
                            for k in range(1, n + 1)])
        for q in samples:
            gap, pair = self.clearance(q)
            count += 1
            if gap < minimum:
                minimum, closest_pair = gap, pair
            if gap <= 0.0:
                raise ValueError(f"Self collision: {pair}, sphere clearance={gap:.6f} m")
        return {"passed": True, "method": "independent_cpu_spheres_densified_linear_segments",
                "samples": count, "max_joint_step_rad": max_step,
                "minimum_clearance_m": minimum, "closest_pair": closest_pair,
                "continuous_collision_proof": False, "hardware_geometry_verified": False}
