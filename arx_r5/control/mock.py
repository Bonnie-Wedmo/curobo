"""明确标识为模拟的控制器，支持通信故障、陈旧反馈和跟踪误差注入。"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Protocol

from arx_r5.trajectory import vector


class Clock(Protocol):
    """只供执行器调度测试的时钟接口。"""

    def now(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class VirtualClock:
    """默认虚拟时钟，也用于精确注入阻塞耗时而无需真实 sleep。"""

    def __init__(self) -> None:
        self.elapsed = 0.0

    def now(self) -> float:
        """返回模拟单调时间。"""
        return self.elapsed

    def sleep(self, seconds: float) -> None:
        """推进虚拟时间。"""
        self.elapsed += max(0.0, seconds)


class MonotonicClock:
    """普通系统调度，不宣称硬实时。"""

    def now(self) -> float:
        """读取系统单调时钟。"""
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        """等待到下一轮截止时间。"""
        time.sleep(max(0.0, seconds))


@dataclass
class Feedback:
    """控制端统一的反馈；时间来自本进程单调时钟。"""

    position: list[float]  # 六关节位置 rad。
    velocity: list[float]  # 六关节速度 rad/s。
    received_at: float  # 接收时间，不冒充硬件采样时间。
    sequence: int  # Mock 序列号；官方接口暂未提供。
    source: str = "mock"  # 永远不标记为硬件反馈。


class MockController:
    """瞬时精确跟踪的协议模拟；不模拟电机、动力学或真实通信延时。"""

    def __init__(self, initial: list[float], fail_at: int | None = None,
                 stale: bool = False, offset: float = 0.0) -> None:
        self.position = list(vector(initial, 6, "initial"))
        self.velocity = [0.0] * 6
        self.fail_at = fail_at
        self.stale = stale
        self.offset = offset
        self.commands = 0
        self.sequence = 0
        self.stopped = False

    def read_state(self, now: float) -> Feedback:
        """返回显式标识的 Mock 状态。"""
        if not self.stale:
            self.sequence += 1
        return Feedback(self.position.copy(), self.velocity.copy(),
                        now - 10.0 if self.stale else now, self.sequence)

    def send(self, position: list[float], velocity: list[float], now: float) -> None:
        """模拟接受一个时刻的目标，不包含真机 SDK 调用。"""
        if self.stopped:
            raise RuntimeError("Mock controller already stopped")
        if self.fail_at is not None and self.commands == self.fail_at:
            raise ConnectionError("Injected MOCK communication failure")
        self.position = [x + self.offset for x in vector(position, 6, "position")]
        self.velocity = list(vector(velocity, 6, "velocity"))
        self.commands += 1

    def stop(self) -> bool:
        """只在 Mock 中能确认停止；不代表官方 SDK 的停止效果。"""
        self.stopped = True
        self.velocity = [0.0] * 6
        return True
