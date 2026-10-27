"""官方 R5 Python SDK 的受阻适配框架；不自动导入或实例化 SDK。"""
from __future__ import annotations

from typing import Any

from arx_r5.common import load_profile
from arx_r5.trajectory import vector


SDK_CONTRACT = {
    "repository": "https://github.com/ARXroboticsX/R5",
    "commit": "e87d09c11edb65f6ae672abc7a41fe2277a2f12f",
    "constructor": "arx_r5_python.InterfacesPy(urdf_path, can_port, type)",
    "wrapper_initialization": "SingleArm calls arx_x(500, 2000, 10); semantics undocumented",
    "position_command": "set_joint_positions(six_positions), then set_arm_status(5)",
    "position_feedback": "get_joint_positions() -> vector<double>[7] per InterfacesPy.hpp",
    "velocity_feedback": "get_joint_velocities() -> vector<double>[7] per InterfacesPy.hpp",
    "mode_2": "PROTECT / damping (not verified emergency stop or position hold)",
    "mode_1": "GO_HOME: motion command, never use as emergency stop",
    "controller_period_s": None,
    "watchdog_timeout_s": None,
    "safe_stop_verified": False,
}


def decode_sdk_feedback(positions: list[float], velocities: list[float]) -> dict[str, Any]:
    """解析头文件规定的 7 维数据，保留夹爪；映射仍需实机逐轴验证。"""
    vector(positions, 7, "SDK positions")
    vector(velocities, 7, "SDK velocities")
    return {"arm_position_raw": positions[:6], "arm_velocity_raw": velocities[:6],
            "gripper_position_raw": positions[6], "gripper_velocity_raw": velocities[6],
            "joint_mapping_verified": False, "feedback_freshness_verified": False}


class OfficialR5Adapter:
    """硬件入口在构造 SDK 前拒绝执行，不能用轨迹内布尔标志解除。"""

    def __init__(self) -> None:
        self.contract = SDK_CONTRACT.copy()

    def initialize(self) -> None:
        """初始化可能启动力矩/后台线程；在依据不足时禁止连接，包括读状态。"""
        raise RuntimeError("HARDWARE BLOCKED before SDK construction: "
                           + "; ".join(load_profile()["hardware_blockers"]))

    def read_state(self) -> None:
        """待确认初始化副作用、SDK 映射和反馈新鲜度后接入官方读取函数。"""
        self.initialize()

    def move_to_start(self, target: list[float]) -> None:
        """预留 demo 前的真机准备入口；当前始终拒绝，不下发目标角度。"""
        vector(target, 6, "START in URDF radians")
        # 官方 SingleArm.set_joint_positions 仅调用 set_joint_positions + set_arm_status(5)。
        # 不能直接传入 URDF START：还缺 SDK 映射、受检过渡轨迹和反馈到位确认。
        # go_home() 的 status=1 是厂商回零命令，与本项目 START 不是同一目标。
        self.initialize()
        # 即使今后放开连接，也不能把“初始化成功”当作“准备动作完成”。
        raise RuntimeError("Move-to-START is not implemented/commissioned: require mapped live "
                           "state, validated current-to-START trajectory, timed execution, "
                           "verified stop and feedback-confirmed arrival")

    def execute(self, trajectory: dict[str, Any], operator_authorized: bool = False) -> None:
        """操作者授权是必要条件但不能替代限位、时序、急停及模型验证。"""
        if not operator_authorized:
            raise RuntimeError("Explicit operator execution authorization is missing")
        self.initialize()

    def stop(self) -> None:
        """无可证明的停止语义，拒绝将停止发送或 PROTECT 返回值当成安全停止。"""
        raise RuntimeError("No commissioned hardware safe-stop implementation")
