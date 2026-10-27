"""不依赖 cuRobo 的配置、来源摘要和 JSON 工具。"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


def load_profile() -> dict[str, Any]:
    """读取本地可信离线限制，不采用轨迹文件自己提供的限值。"""
    return json.loads((ROOT / "config/offline_profile.json").read_text())


def digest(path: Path) -> str:
    """返回文件 SHA256。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def content_digest(data: Any) -> str:
    """对 JSON 内容计算与缩进无关的摘要，用于绑定状态、计划和授权文件。"""
    content = json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(content.encode()).hexdigest()


def provenance() -> dict[str, str]:
    """同时绑定运动学、网格、碰撞球和规划参数，路径可跨机器。"""
    paths = [ROOT / "config/offline_profile.json", ROOT / "config/robot.yml",
             ROOT / "models/r5_offline.urdf"]
    paths += sorted((ROOT / "models/meshes").glob("*.STL"))
    return {str(p.relative_to(ROOT)): digest(p) for p in paths}


def write_json(path: Path, data: Any) -> None:
    """拒绝 NaN/Infinity，原子替换完整 JSON；不会留下半条轨迹。"""
    content = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content)
    temporary.replace(path)
