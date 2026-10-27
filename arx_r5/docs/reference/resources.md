# 官方资源、参数与阻塞项

## 固定来源

检查日期：2026-10-19。逐文件 SHA256、完整提交号和链接保存在 [sources.json](sources.json)。

| 资源 | 固定版本 / 本地对应 |
|---|---|
| [ARX_Model](https://github.com/ARXroboticsX/ARX_Model/tree/74386441845f12af4da203843b6400951d6c5530/R5/R5a) | `74386441845f12af4da203843b6400951d6c5530`；原 URDF 和九个 STL 在 `models/` |
| [R5 SDK](https://github.com/ARXroboticsX/R5/tree/e87d09c11edb65f6ae672abc7a41fe2277a2f12f) | `e87d09c11edb65f6ae672abc7a41fe2277a2f12f`；原 `R5_master.urdf` 在 `models/upstream/` |
| 本地 cuRobo | Git `c0a1f5f5dca474235045844e8bcf4869e403860d`，`v0.8.0-43-gc0a1f5f` |
| [cuRobo 文档](https://curobo.org/) | 背景资料；API 以本地源码及实际 GPU 调用为准 |

ARX 模型快照上游没有根目录许可证文件；保留来源，不能把 SDK 的 BSD 许可证擅自解释为模型资产的再分发许可。SDK 许可证另存于 `models/upstream/R5-SDK-LICENSE`。

## 官方文档已确认的参数

下表来自 [03-ROS2-单臂R5-SDK.pdf](https://github.com/ARXroboticsX/R5/blob/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/00-readme/03-ROS2-单臂R5-SDK.pdf) 的关节限位表；ROS1 与双臂说明中亦有一致数值。此处的“确认”指文件证据，不代表用户这台硬件的标定已完成。

| 关节 | 位置范围 rad | R5a URDF 关节局部轴 |
|---|---|---|
| joint1 | −3.14 … 2.6 | +Z |
| joint2 | −3.6 … 0.1 | +Y |
| joint3 | −1.57 … 1.57 | +Y |
| joint4 | −1.3 … 1.3 | +Y |
| joint5 | −1.57 … 1.57 | +Z |
| joint6 | −2.1 … 2.1 | +X |

六个 revolute 关节参与规划；R5a 另有两个 prismatic 夹爪关节，范围 0–0.044 m，沿 ±Y，没有 mimic 标签。官方反馈说明是“六个关节＋夹爪”七维，并不是把两个 URDF 滑块分别控制的八维接口。

手册规定轴向正转遵循右手定则；URDF 中关节轴处于各自关节坐标，不是统一世界坐标。joint3 的 ±π 原点旋转也必须参与变换。实物电机方向、编码器零位、SDK 偏置与 URDF 零位的对应关系未得到逐轴实测。

文档中关节位置以 rad 表述。Python wrapper 原样转发向量，没有单位换算代码；底层库源码未公开，速度反馈单位不能仅凭函数名证明。夹爪命令另有 0–5 与 0–80 mm 的说明，不能将它等同于 URDF 的单指位移。

## 两份模型的具体差异

| 项目 | R5a | R5_master |
|---|---|---|
| joint3 origin rpy | (−3.1416, 0, 0) | (+3.1416, 0, 0) |
| joint4 origin xyz m | (0.245, −0.00005, −0.06) | (0.245, 0, −0.06) |
| joint5 origin xyz m | (0.073914, 0.00005, −0.083391) | (0.06775, −0.00005, −0.0865) |
| joint6 origin xyz m | (0.025286, 0, 0.083391) | (0.02895, 0, 0.0865) |
| link6 质量 kg | 0.44089 | 0.281000 |
| 两片夹爪 | link7/link8，joint7/joint8 | 不存在 |
| mesh URI 包名 | R5a | X5liteaa0 |

joint1/2 的父子连接、原点与轴相同；六个主动关节的轴表达相同。±3.1416 只是近似 ±π，因此仍有约 1.47e−5 rad 的微小姿态差。腕部位置差不能这样消除：三个测试状态的 `base_T_link6` 平移差约 2.50–3.16 mm。link1…6 的质心、惯量和部分质量亦不同，机器可读报告逐项列出。

公开仓库没有说明这些差异对应的硬件修订、夹爪版本或变更原因。目前无法证明模型等价，也没有进行坐标“修正”或模型混合。R5_master 的引用网格路径本身不能证明这些网格与 R5a 相同。

## Python 与 ROS2 控制接口比较

| 项目 | Python SDK | ROS2 SDK |
|---|---|---|
| 构建 | pybind11 + CMake + 厂商 `.so` | 另外需要 ROS2、消息包、节点 |
| 六轴位置目标 | `set_joint_positions` 后 wrapper 切 mode 5 | `RobotCmd.joint_pos[6]` + mode 5 |
| 状态 | 三个 getter；头文件约定七维 vector | `RobotStatus` 的三个七元素数组，节点加 ROS 时间戳 |
| 定时语义 | 没有传入时间戳/周期的轨迹 API | 订阅单次目标，不是 FollowJointTrajectory action |
| 已见频率 | wrapper 的 `dt=0.05` 只是成员值 | 状态发布定时器 1 ms，不是电机执行周期保证 |
| 持续轨迹 | 需自行调度，并实测底层跟踪/保持 | 同样需调度与实测 |
| 中断 | 有 mode 2 接口，没有停止状态确认 | shutdown reset 对象，实际停止行为在二进制库 |
| 当前结论 | 优先保留 Python 框架 | 不作为绕过安全缺口的替代方案 |

具体证据：

- [single_arm.py](https://github.com/ARXroboticsX/R5/blob/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/py/ARX_R5_python/bimanual/script/single_arm.py)：构造 `InterfacesPy` 后调用 `arx_x(500,2000,10)`，参数语义没有公开注释；不能猜作速度、频率或加速度。`set_joint_positions()` 声明 bool，但 wrapper 没有返回底层执行结果。
- [InterfacesPy.hpp](https://github.com/ARXroboticsX/R5/blob/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/py/ARX_R5_python/bimanual/lib/arx_r5_src/include/arx_r5_src/interfaces/InterfacesPy.hpp)：声明七维反馈；位置接口旁仍有 `useless` 注释，与 wrapper 使用情况不一致。pimpl 实现在 `.so`，未见定时执行源码。
- [R5Controller.cpp](https://github.com/ARXroboticsX/R5/blob/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/ROS2/R5_ws/src/ARX_R5_ros2_V7/arx_r5_controller/src/R5Controller.cpp)：`PubState` 与 `VrPubState` 用 `i <= 7` 写入长度 7 的数组，并访问七维反馈的第八个元素，存在越界问题。`RobotCmd` 回调每次还调用夹爪命令，不能为“不控制夹爪”随意发送默认值。
- 同一源码和 Python wrapper 按 `type/end_type` 分支选择 `X5liteaa0.urdf` 或 `R5_master.urdf`；默认值为 0。不能仅凭文件名或非零分支推断用户的 R5 应使用哪个值。
- `InterfacesThread.hpp` 有 `getErrorCode()`，但检查到的 Python binding 未绑定该接口；Python 无可见硬件采样时间戳、序号或明确通信健康字段。
- 手册的 mode 0 是力矩清零，1 是回初始位形，2 是加阻尼，3 是重力补偿，4/5 是末端/关节控制。mode 2 不等于保持姿态的急停。错误处理手册还描述保护后可能垂落，必须由厂家明确适用的停止和支撑措施。

## 尚未解决的真机阻塞项

已检索 Python、ROS1、ROS2、双臂手册及公开配置、头文件、控制源码。以下是公开证据或现场验证仍缺失的项目，不是要求用户重复查已给出的限位表。

1. 两种腕部模型差异的原因、用户硬件/夹爪版本，以及 SDK 模型分支选择依据。
2. 每轴可靠的速度、加速度上限，以及载荷依赖条件；URDF `velocity=1000` 不可用。
3. SDK 对每轴位置/速度的单位、顺序、符号和零位映射实测；夹爪实际开口与几何状态。
4. SDK 初始化副作用、`arx_x` 三参数定义、内部位置插值、指令覆盖/排队、实际更新周期。
5. 反馈刷新与陈旧数据识别、跟踪误差阈值、超时、CAN 断线与进程异常退出行为。
6. 有明确停止确认的安全动作、急停设备和断电支撑办法；不能使用回零代替停止。
7. 实物底座安装夹具、线缆、负载等几何；当前示例最小球间隙仅约 0.279 mm，不能当作真机安全间隙。
8. 控制计算机 OS/架构、SDK 构建、CAN 接口及已采集的真实起点，现场操作者执行授权。

对应阻塞集中保存在 `config/offline_profile.json`。当前 `OfficialR5Adapter` 在 SDK 构造前硬拒绝，因此第二阶段真机验收尚未开始。
