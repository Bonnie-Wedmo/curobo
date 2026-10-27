# 跨计算机部署与真机验证

目标：在控制计算机复核 v2 规划文件，完成 Mock 流程；待官方参数与现场验证齐备后，再开发和验收真实执行路径。

旧 v1 轨迹及状态示例需重新生成；配置升级补充了相邻肘部躯体检查。

## 当前可以执行的操作

1. 规划端使用现有 `.venv`，运行 [README](../../README.md) 的离线闭环命令。不重装 cuRobo、不升级 PyTorch/CUDA。
2. 将整个 `arx_r5/` 目录复制到控制计算机的工作目录中。轨迹 JSON 不依赖规划机绝对路径；控制端需要本地模型、碰撞球、参数文件以重新验证 SHA256 和自碰撞。
3. 控制机仅读取轨迹时使用标准库；完整 Mock 及碰撞复核需要 numpy、SciPy、PyYAML、trimesh，不需要 GPU、PyTorch 或 cuRobo。推荐使用独立 Python 3.11 环境，依赖版本见 `artifacts/environment.json`。不要在未知 SDK 环境里直接升级依赖。
4. 从包含 `arx_r5/` 的目录运行：

   ```bash
   python -S -m arx_r5.trajectory arx_r5/artifacts/visual_demo_trajectory.json
   python -m arx_r5.control.run arx_r5/artifacts/visual_demo_trajectory.json
   python -m arx_r5.control.run arx_r5/artifacts/visual_demo_trajectory.json \
     --realtime --log arx_r5/artifacts/mock_realtime.jsonl
   ```

5. 运行故障注入并检查日志。下面命令应返回非零退出码，日志应在第 5 个已成功下发的点后记录中止：

   ```bash
   python -m arx_r5.control.run arx_r5/artifacts/visual_demo_trajectory.json \
     --fail-at 5 --log arx_r5/artifacts/mock_fault.jsonl
   ```

6. 查看 SDK 阻塞诊断；硬件模式预期拒绝，不会初始化设备：

   ```bash
   python -m arx_r5.control.run --diagnose-sdk
   python -m arx_r5.control.run --mode hardware
   ```

如 GPU 在沙箱中不可用、但主机 `nvidia-smi` 和 `.venv` 的 `torch.cuda.is_available()` 正常，应在有 GPU 访问权限的主机终端运行。没有 GPU 时仍可运行文件读取、CPU 几何复核和 Mock；不能把它们称作 GPU 规划成功。

## 生成控制机准备报告

在实际控制机的目标 Python 环境运行以下命令；SDK 路径替换为本机已下载的官方 R5 仓库：

```bash
python -m arx_r5.control.diagnose --sdk-root /path/to/R5 \
  --output arx_r5/artifacts/control_host_diagnostics.json
```

核对报告中的 SDK 提交和源码摘要、Python ABI、ELF 架构、动态依赖以及 CAN 接口。
这是被动诊断，不导入 SDK，也不创建 CAN 对象或配置接口。没有 SDK 源码时可省略
`--sdk-root`，报告会列出缺失项。当前仓库自带的报告来自规划机，并非你的控制机验收记录。

## 用状态快照演练规划与执行

1. 从控制端导出明确标记的 Mock 快照：

   ```bash
   python -m arx_r5.control.state mock \
     --position 0 0 1.1 -1.0 0.13 0.06 \
     --output arx_r5/artifacts/mock_start_snapshot.json
   ```

2. 将快照交给规划端，绑定采集来源与完整状态。以下目标含肘、腕协调变化，仍只用于离线：

   ```bash
   python -m arx_r5.planning.plan \
     --start-snapshot arx_r5/artifacts/mock_start_snapshot.json \
     --goal 0 0 0.45 -0.35 0.13 0.06 \
     --output arx_r5/artifacts/snapshot_plan.json
   ```

   不需要重新规划时，使用已生成的 `artifacts/visual_demo_trajectory.json`；
   它是直接指定关节角的离线示例；需要绑定起点来源时，使用上面的 `--start-snapshot` 命令重新生成。

3. 将轨迹送回控制端，复核后执行 Mock：

   ```bash
   python -S -m arx_r5.trajectory arx_r5/artifacts/visual_demo_trajectory.json
   python -m arx_r5.control.run arx_r5/artifacts/visual_demo_trajectory.json \
     --initial-snapshot arx_r5/artifacts/mock_start_snapshot.json \
     --realtime --log arx_r5/artifacts/deployment_mock_realtime.jsonl
   ```

4. 生成部署预检报告：

   ```bash
   python -m arx_r5.control.preflight arx_r5/artifacts/visual_demo_trajectory.json \
     --current-state arx_r5/artifacts/mock_start_snapshot.json \
     --commissioning arx_r5/config/commissioning.template.json \
     --output arx_r5/artifacts/deployment_preflight.json
   ```

   预期退出码为 2，但 JSON 报告应正常生成。数值和离线几何可通过；Mock 来源、未知现场参数、
   停止能力和硬件执行路径应被标为失败或未验证。不要将这份报告理解成可执行许可。

## 导入现场已有记录

在下文现场步骤完成状态采集与关节标定后，才使用本节处理已有文件；本模块不提供会自动连接
设备的采集脚本。原始记录和映射字段见[部署接口参考](../reference/deployment_interface.md)。

1. 复制 `config/calibration.template.json` 为现场标定文件，填写序列号、SDK 版本、六轴排列、
   方向、零位、单位与实测证据。不要默认使用恒等映射，也不要修改原始采集时间。
2. 将原始七维反馈记录按约定写成 JSON，执行导入：

   ```bash
   python -m arx_r5.control.state import \
     --recording /path/to/recorded_sdk_state.json \
     --calibration /path/to/measured_calibration.json \
     --output arx_r5/artifacts/imported_start_snapshot.json
   ```

3. 将导入快照传给规划端的 `--start-snapshot`。夹爪状态与模型不符或起点仍在运动时会拒绝。
   记录导入成功仍不证明状态实时有效；目前输出仍是基于离线模型的测试轨迹。
4. 将 `commissioning.template.json` 复制为现场配置，按厂家依据和实测填写限值、几何、
   控制与停止条件，各区段关联证据文件。用执行前重新采集的快照运行预检，不能复用规划前
   的旧状态来假定机器人未移动。跨机 UTC 时钟及设备反馈新鲜度都需要验证。
5. 若现场条件不满足，保留报告用于定位。即使所有可由文件检查的项目通过，当前真实执行后端
   仍未实现，不能通过填写授权文件或更改 `hardware_eligible` 来启用。

## 更新离线模型

修改 `config/offline_profile.json` 后必须重新生成模型并重新规划，旧文件会因哈希不同被拒绝：

```bash
python -m arx_r5.planning.model --build
python -m arx_r5.planning.verify_fk
python -m unittest discover -s arx_r5/tests -v
```

球体参数变粗可能产生假碰撞；不能通过自动添加非相邻排除对来消除失败。当前 v2 测试起点已避开旧反向折叠，但模型间隙仍很小，只用于离线验证，不能直接搬到真机。

## 控制机官方 SDK 构建准备

本次没有在规划机安装 SDK，也没有执行厂商示例。SDK 构造可能初始化电机或启动线程，读取状态并非已经证实无动作的操作。

在确认目标 OS/架构、厂商依赖与二进制兼容后，可在控制机拉取本次审查版本并仅构建接口：

```bash
git clone https://github.com/ARXroboticsX/R5.git
git -C R5 checkout e87d09c11edb65f6ae672abc7a41fe2277a2f12f
cd R5/py/ARX_R5_python
# 先确认 pybind11、CMake、编译器和厂商动态库依赖满足。
bash build.sh
source setup.sh
```

这些构建步骤来自上游，未在未知控制机上验收。上游 CMake 为 x86 与 aarch64 选择不同库，需核实实际发布是否包含所需架构文件。`bimanual/__init__.py` 会搜索编译扩展；构建失败时不能仅复制其他 Python 版本的 `.so`。不要直接运行 `test_single_arm.py` 或 `test_keyboard.py` 作为“只读连通性检查”；它们可能发运动指令。CAN 配置脚本会修改系统设置，只能在确认接口和设备归属后由现场人员使用官方 CAN 手册配置。

## 真机试验的必要顺序（当前受阻）

1. 由厂家确认 R5 硬件修订、末端类型、URDF、`type/end_type` 参数、位置/速度/加速度上限及初始化动作；记录证据版本。
2. 确认机械固定、载荷、夹爪开口、工作区、急停和失去力矩时的支撑措施。补充安装板、夹具等几何。
3. 验证厂家规定的停止机制，包括停止确认、通信中断、异常退出与恢复；不能以“停止发送”或 mode 1 回零代替。
4. 在现场授权和安全措施下建立连接，先验证反馈维度、健康状态、新鲜度和单位。逐轴小范围核验名称、符号、零位，建立明确的 SDK↔URDF 映射；记录真实状态及时间，而不是采用示例 `q_start`。
5. 单独验证位置目标的覆盖/排队、插值/保持、允许周期、延迟和跟踪误差。Python 的 `sleep` 不提供硬实时保证；必要时使用厂家批准的执行器或控制进程。
6. 在适配层实现并审查真实反馈采集、调度、watchdog、错误状态与停止确认。解除硬拒绝需要代码变更和硬件验收，不能仅把 `hardware_eligible` 改成 true。
7. 把真实静止关节状态传给规划端，生成经过真实限制和几何复核的新轨迹。控制端重新采样当前状态，与起点比较；不一致则重新规划。
8. 操作者针对这一次轨迹明确授权后，再执行经厂家参数约束的小范围低速试验。记录目标、实际位置/速度、实际时间、错误状态、停止请求与停止确认，计算跟踪 RMS/峰值误差。
9. 验证正常完成、人工中止、反馈超时、CAN 中断和跟踪超差。通过前不得声称真机阶段验收完成。

当前仍缺少第 1–5 步的必要证据与实测，因此本工程的真实硬件入口保持禁止执行。
