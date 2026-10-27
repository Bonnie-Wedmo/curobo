# 文件与接口参考

## 代码位置

| 路径 | 职责 |
|---|---|
| `config/offline_profile.json` | 参数来源、离线限位、固定夹爪状态、硬件阻塞项 |
| `config/robot.yml` | 1544 个原始球 + 793 个肘部躯体别名球，固定别名与碰撞对配置 |
| `models/upstream/` | 两份未修改的官方 URDF |
| `models/r5_offline.urdf` | 路径可搬迁、位置范围来自手册、速度仅供离线的派生模型 |
| `models/meshes/` | R5a 官方九个 STL |
| `planning/model.py` | 模型生成、差异分析、独立 CPU FK |
| `planning/verify_fk.py` | GPU/CPU/yourdfpy FK 对照及 GPU 碰撞正反例 |
| `planning/collision.py` | 独立 CPU 球体碰撞及关节段加密验证 |
| `planning/plan.py` | `MotionPlanner.plan_cspace`、夹爪校验、六轴结果提取 |
| `trajectory.py` | 标准库数值验证、JSON 读取；导出时另做几何复核 |
| `planning/visualize.py` | 三组关节曲线、模型球体图、自动取景、末端路径与 Viser 回放 |
| `planning/demo.py` | 与 UR10e 同款显式 GPU 规划、轨迹提取、原生 ViserVisualizer、逐帧 set_joint_state |
| `control/mock.py` | Mock 反馈、指令接受与故障注入 |
| `control/run.py` | 控制端验证、Mock 调度、停止、执行日志与误差统计 |
| `control/sdk.py` | 官方 SDK 接口契约、七维解码、连接前阻塞 |
| `control/state.py` | Mock 快照、现有 SDK 记录按显式标定导入、规划起点绑定 |
| `control/diagnose.py` | 主机、SDK 来源、ELF 依赖及 CAN 接口的被动诊断 |
| `control/preflight.py` | 本地证据、轨迹与当前状态逐项预检，不连接设备 |
| `planning/acceptance.py` | 16 个逐轴/耦合案例，保留规划或碰撞失败 |
| `config/calibration.template.json` | 未填写的关节映射模板，无默认恒等映射 |
| `config/commissioning.template.json` | 未填写的现场验收参数与证据模板 |
| `tests/test_safety.py` | 文件损坏、限位、假通过标记、Mock 故障及硬件拒绝测试 |
| `tests/test_deployment.py` | 状态来源、证据绑定、预检、调度超时及停止失败测试 |

## 轨迹 JSON v1

所有向量采用 `joint1 … joint6` 顺序。位置 rad，速度 rad/s，加速度 rad/s²，时间 s。`T` 是插值后有效轨迹点数，不能使用预分配缓冲区大小替代。

| 字段 | 类型与意义 |
|---|---|
| `schema_version` | 整数 1 |
| `robot_model` | `ARX_R5_R5a_offline`，明确表示离线模型 |
| `joint_names` | 固定六名称顺序，禁止静默猜测或重排输入文件 |
| `start_state`, `goal_state` | 长度 6 的有限数值，接受端点误差最多 0.001 rad |
| `start_state_source` | `offline_test` 或 `hardware_snapshot_unverified`，后者必须绑定导入记录 |
| `start_snapshot`, `start_snapshot_sha256` | 可选完整起点快照及规范化 JSON 内容摘要；详见[部署接口](deployment_interface.md) |
| `time_from_start` | 长度 T，从 0 严格递增；相邻时间差与 `interpolation_dt` 一致 |
| `joint_positions` | T×6 |
| `joint_velocities` | T×6，直接保留 cuRobo 插值导数 |
| `joint_accelerations` | T×6；本版本要求存在以完成加速度检查 |
| `planning_status` | `success`，失败不会产生新的成功文件 |
| `joint_limits_validation` | 数值检查结果、使用参数的来源、峰值；`hardware_limits_verified=false` |
| `self_collision_validation` | 球体检查方式、加密步进、样本数、最小间隙；`continuous_collision_proof=false` |
| `configuration_version` | 当前 `arx-r5-offline-v2-elbow-body-guard`；v1 文件需重新规划 |
| `provenance` | 本地配置、派生 URDF、九个 STL 的 SHA256 映射，键为相对路径 |
| `created_at_utc` | ISO 8601 UTC 时间 |
| `units` | 四项明确单位标识 |
| `locked_gripper` | joint7/joint8 固定值，单位 m |
| `interpolation_dt` | 0.025 s，来自实际求解器配置 |
| `controller_period` | null，尚无真实控制周期依据 |
| `planner` | API、包版本、实际导入路径、CUDA graph 状态、优化步长、求解耗时 |
| `hardware_eligible` | 必须为 false；本适配器无资格认证路径 |
| `hardware_blockers` | 未解决项目说明 |
| `task_space_diagnostics` | `demo.py` 输出 link6 两端变换、平移、朝向漂移、路径长度及直线偏差；不代表已施加笛卡尔约束 |

`load_trajectory()` 返回普通 dict，数值检查只依赖 Python 标准库。它不把 JSON 中的 `passed=true` 当成新一次几何验证；控制入口和导出入口会重算碰撞。哈希用于防止配置意外混用，不是数字签名或厂家认证。

当前格式要求等周期、至少三个点、首尾静止。它不执行自动重新采样，也不修改不合格轨迹的端点。需要其他控制周期时，必须重新生成并验证全部导数和碰撞，不能只修改时间戳。

## 控制日志

JSONL 每行是独立事件：`started`、`feedback`、`finished`。反馈事件记录计划时间、实际调度时间、目标、模拟实际位置/速度、逐轴误差和 `source=mock`。

同名 `.summary.json` 记录状态 `completed/aborted/stop_failed`、已发指令数、逐轴 RMS 和最大绝对位置误差、Mock 停止确认。`completed` 要求最终反馈接近目标且静止；发送完所有点本身不代表完成。停止异常写入 `stop_error`，保留原始执行异常；超差反馈计入误差统计。

调度器在反馈读取和指令发送返回后重新计时；超时后中止，不集中补发积压点。反馈序号必须递增。以上行为仅在 Mock 控制器上验证，不提供可复用到真机的控制时序保证。

Mock 检查的 0.1 s 新鲜度/迟到阈值、0.01 rad 跟踪阈值、0.001 rad 终点及起点阈值仅是软件测试设置。它们不是官方控制规范，不能复制到真机后宣称安全。

## 官方 SDK 框架

`OfficialR5Adapter.initialize/read_state/execute/stop` 均保留明确的拒绝行为；初始化在导入 SDK、创建 CAN 对象或启动线程前结束。`execute(..., operator_authorized=True)` 也不会绕过阻塞。当前没有实现实际硬件指令发送、停止确认或状态采集。

`decode_sdk_feedback()` 只解析公开头文件约定的七维位置与速度，分别保留前六轴和夹爪原始值。它标记 `joint_mapping_verified=false` 和 `feedback_freshness_verified=false`，不会擅自宣布单位、零位或新鲜度正确。
