"""控制机只读诊断：检查 SDK 文件/ABI/CAN 元数据，不加载厂商动态库。"""
from __future__ import annotations

import argparse
import importlib.util
import json
import platform
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

from arx_r5.common import ROOT, digest, write_json
from arx_r5.control.sdk import SDK_CONTRACT


def run_readonly(arguments: list[str]) -> dict:
    """有限超时地运行已知只读系统工具，不经 shell 解释。"""
    try:
        result = subprocess.run(arguments, capture_output=True, text=True, timeout=10)
        return {"returncode": result.returncode, "stdout": result.stdout[:16000],
                "stderr": result.stderr[:2000]}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)}


def diagnose(sdk_root: Path | None = None, can_interface: str | None = None) -> dict:
    """报告环境和缺失依赖；不把接口存在或编译成功标为通信成功。"""
    suffix = sysconfig.get_config_var("EXT_SUFFIX")
    report = {"kind": "arx_r5_control_host_diagnostics", "read_only": True,
              "hardware_connected_tested": False, "sdk_imported": False,
              "python": sys.version, "executable": sys.executable,
              "architecture": platform.machine(), "platform": platform.platform(),
              "python_extension_suffix": suffix,
              "tools": {name: shutil.which(name) for name in ("cmake", "c++", "readelf", "ip", "git")},
              "python_modules": {name: importlib.util.find_spec(name) is not None for name in
                                 ("pybind11", "numpy", "yaml", "scipy", "trimesh")},
              "sdk_contract": SDK_CONTRACT, "blockers": []}
    if sdk_root is None or not sdk_root.is_dir():
        report["blockers"].append("SDK checkout not supplied/found; use --sdk-root with the R5 repository root")
    else:
        base = sdk_root / "py/ARX_R5_python/bimanual"
        commit = run_readonly(["git", "-C", str(sdk_root), "rev-parse", "HEAD"])
        report["sdk_git"] = commit
        if commit.get("stdout", "").strip() != SDK_CONTRACT["commit"]:
            report["blockers"].append("SDK commit differs from reviewed version; re-review required")
        source_records = json.loads((ROOT / "docs/reference/sources.json").read_text())
        comparisons = []
        for record in source_records:
            if record["repository"] == "R5":
                file = sdk_root / record["path"]
                matches = file.is_file() and digest(file) == record["sha256"]
                comparisons.append({"path": record["path"], "matches_reviewed_source": matches})
        report["source_integrity"] = comparisons
        if not all(item["matches_reviewed_source"] for item in comparisons):
            report["blockers"].append("Some reviewed SDK files are missing or modified")
        libraries = []
        for file in sorted((base / "lib/arx_r5_src").glob("*.so")):
            entry = {"path": str(file), "sha256": digest(file)}
            if report["tools"]["readelf"]:
                # readelf 解析文件；不使用可能加载/执行动态库的方式做诊断。
                entry["elf_header"] = run_readonly(["readelf", "-h", str(file)])
                entry["dynamic_dependencies"] = run_readonly(["readelf", "-d", str(file)])
            libraries.append(entry)
        report["vendor_libraries"] = libraries
        extensions = sorted((base / "api").rglob("arx_r5_python*.so")) if (base / "api").exists() else []
        report["built_extensions"] = [{"path": str(file), "sha256": digest(file),
                                       "python_abi_filename_matches": file.name.endswith(suffix)}
                                      for file in extensions]
        if not any(file.name.endswith(suffix) for file in extensions):
            report["blockers"].append("No built Python extension matching this interpreter ABI")
        if not report["python_modules"]["pybind11"]:
            report["blockers"].append("pybind11 absent from this Python environment; CMake/system installation may differ")
    interfaces = []
    for path in sorted(Path("/sys/class/net").glob("*")):
        try:
            if (path / "type").read_text().strip() == "280":  # Linux ARPHRD_CAN。
                interfaces.append({"name": path.name, "operstate": (path / "operstate").read_text().strip()})
        except OSError:
            continue
    report["can_interfaces"] = interfaces
    if can_interface:
        if not any(item["name"] == can_interface for item in interfaces):
            report["blockers"].append("Requested CAN interface not found as a Linux CAN device")
        if report["tools"]["ip"]:
            report["requested_can_link"] = run_readonly(["ip", "-details", "-json", "link", "show", "dev", can_interface])
    elif not interfaces:
        report["blockers"].append("No Linux CAN interfaces visible")
    report["blockers"].append("SDK initialization, feedback freshness and safe stopping require hardware commissioning")
    return report


def main() -> None:
    """生成可带回规划机分析的 JSON 报告；始终不操作 CAN。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path)
    parser.add_argument("--can-interface")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/control_host_diagnostics.json")
    args = parser.parse_args()
    report = diagnose(args.sdk_root, args.can_interface)
    write_json(args.output, report)
    print(f"Read-only report: {args.output}")
    for blocker in report["blockers"]:
        print(f"Unresolved: {blocker}")


if __name__ == "__main__":
    main()
