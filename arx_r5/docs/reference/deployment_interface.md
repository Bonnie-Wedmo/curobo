# 状态交换、现场参数与预检参考

这些接口只处理文件与离线诊断，不初始化 SDK。操作命令见[部署指南](../howto/deployment.md)。

## 关节状态快照 v1

`control.state` 产生 `kind=arx_r5_joint_snapshot`、`schema_version=1` 的 JSON。

| 字段 | 约束 |
|---|---|
| `robot_model`, `robot_serial` | 型号固定 `ARX_R5`；Mock 标识 `MOCK-ONLY` |
| `source` | `mock` 或 `hardware_recording`，均不是已验证的现场实时反馈 |
| `joint_names` | `joint1` 至 `joint6` |
| `position`, `velocity` | 六个有限数值；单位必须明确为 rad、rad/s |
| `captured_at_utc` | 带时区的 ISO 8601 采集时间；不以文件导入时间替换 |
| `gripper_model_positions` | URDF 中 joint7、joint8 的位置，单位 m；绑定规划时必须等于固定夹爪配置 |
| `sdk_commit` | 导入原始记录时保留的 SDK 版本 |
| `calibration_sha256`, `raw_recording_sha256` | 导入标定和原始记录的文件字节 SHA256 |
| `feedback_health`, `device_timestamp`, `feedback_sequence` | 原始采样元数据；缺失保持未知 |
| `physical_state_verified`, `hardware_eligible` | 导入器均写 false |

`plan --start-snapshot` 与 `--start` 互斥。快照位置必须与请求起点完全相同，速度须低于
离线静止阈值 0.001 rad/s。该阈值不是厂家规范。嵌入完整快照后，校验器同时检查来源、
六轴状态、夹爪与 `start_snapshot_sha256`，旧版没有快照的离线文件仍可读取。

## 已有 SDK 记录导入与标定

原始记录字段：`schema_version=1`、`kind=arx_r5_sdk_recording`、`robot_model=ARX_R5`、
`robot_serial`、`sdk_commit`、`captured_at_utc`、七维 `joint_positions` 和
`joint_velocities`、`units`。第七项单独保存在 `gripper_raw`，不会作为第七个机械臂关节。
`units` 仅允许 rad/rad/s 或 deg/deg/s，描述前六轴；夹爪原始值的单位没有由此得到认证。
`gripper_model_positions` 必须由独立的夹爪映射或实际测量提供。

标定文件以 [calibration.template.json](../../config/calibration.template.json) 为结构：

| 字段 | 约束 |
|---|---|
| `review_status` | 导入要求 `measured`，模板为 `unconfirmed` |
| `robot_model`, `robot_serial`, `sdk_commit` | 必须与原始记录一致 |
| `joint_names` | 六轴 URDF 顺序 |
| `sdk_indices` | 0…5 的排列，无默认值，不能包含夹爪索引 6 |
| `sign` | 六个 +1 或 −1 |
| `offset_rad` | 六个实测零位偏置 |
| `raw_units` | 与原始记录单位完全一致 |
| `evidence` | 实测记录或厂家文件的路径、SHA256、类型 |

变换为 `q_model[i] = sign[i] * unit_scale * q_sdk[sdk_indices[i]] + offset_rad[i]`；
速度使用相同顺序、符号和单位比例，无位置偏置。导入记录仍标记为物理状态未验证；
当前没有实现从设备采集这份原始记录的程序。

## 现场验收参数

[commissioning.template.json](../../config/commissioning.template.json) 的未知数值全部为 null。

| 区段 | 内容及预检用途 |
|---|---|
| `robot` | 序列号、硬件修订、SDK 和标定摘要，关联规划起点与当前快照 |
| `limits` | 实机位置上下限、速度和加速度上限；独立检查轨迹字段与差分速度/加速度 |
| `model` | 本地模型摘要、夹爪、夹具确认、实测最低间隙及关节限位余量 |
| `controller` | 周期、静止/起点/跟踪阈值、迟到/新鲜度上限，以及插值、watchdog、设备时间与跨机时钟确认 |
| `stop` | 停止程序描述、停止确认、断通信及急停测试结果 |

每个区段均需 `evidence={path, sha256, kind}`；`path` 相对配置文件目录或为绝对路径，
`kind` 为 `manufacturer_document` 或 `hardware_measurement`。摘要只检查内容一致性，
不认证证据真实性。模型碰撞检查仍针对当前离线几何；`fixtures_verified=true` 本身不会增加夹具几何。
真实夹具必须先建模并重新生成验证结果，不能仅填写该布尔值。

## 预检报告与授权范围

报告含 `checks`，每项状态为 `pass`、`fail` 或 `unverified`。空模板表示缺证据；
证据完整但值不符合要求表示失败。轨迹周期与控制周期必须一致，程序不自动重新采样。
`tracking_error_rad`、`max_lateness_s` 等为后续真实执行器的验收输入；文件预检不能证明
执行过程中遵守这些阈值。设备时间、通信健康和序号也必须由真实采样实现持续核验。

可选授权文件字段为 `action=execute_once`、`authorized_by`、`robot_serial`、
`trajectory_sha256`、`current_snapshot_sha256`、`issued_at_utc`、`expires_at_utc`。
它检查明确操作范围和有效期；不是数字签名，也不会实际消费一次性授权或发出指令。
内容摘要采用 `common.content_digest()` 的规范化 JSON SHA256，与文件字节 SHA256 有区别。

当前 `hardware_execution_allowed` 始终为 false，CLI 在成功写出受阻报告后退出 2。
原因包括：离线轨迹未获真机资格、文件快照不等于实时采样、真实 SDK 执行与停止确认未实现。
填写模板、传入授权或改写单个布尔值都不能解除硬件入口拒绝。

## 控制机诊断

`control.diagnose` 记录 Python ABI、架构、工具/依赖可见性、固定版本的源码摘要、
厂商 ELF 头和动态依赖，以及 Linux CAN 接口信息。只使用文件读取、git、readelf 和
接口信息查询，不导入厂商 SDK、不运行 ldd、不修改 CAN、不发送 CAN 帧。
退出 0 仅表示诊断报告生成，不表示所有依赖就绪或设备连接成功。
