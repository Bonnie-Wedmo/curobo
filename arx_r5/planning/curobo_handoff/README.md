# 交给 cuRobo 主机的 R5 映射核对结果

核对日期：2026-10-23。此包包含静态证据，不是已经标定完成的 `--mapping` 配置。

## 结论

`planning_demo.py` 足以运行模型内部的离线关节规划，但不包含 SDK → 规划模型的完整映射。能输出轨迹只说明给定模型和配置下的规划成功，不能反向证明 SDK 坐标与模型坐标相同。

本地脚本没有 `--mapping` 参数；另一台主机所说的参数属于其他脚本或后续版本。先在该主机定位参数解析及配置读取代码，按其实际 schema 转换本包证据，不能直接把本包 JSON 当作它的 mapping 文件。

## 当前脚本实际需要什么

| 内容 | 本地状态 |
|---|---|
| 六轴模型关节顺序 | 导入 `arx_r5.common.JOINT_NAMES`，并检查 planner.joint_names；现有轨迹为 joint1…joint6 |
| 起点、终点 | 写死的 START/GOAL，单位按轨迹为 rad；没有读取 SDK 实时关节状态 |
| 模型、碰撞、限位 | 依赖 `arx_r5` 包、config/robot.yml、config/offline_profile.json、models/r5_offline.urdf、网格；本工作区未找到这些配套文件 |
| 规划 API | MotionPlanner.plan_cspace，输入模型关节角，不输入 SDK 笛卡尔目标 |
| 末端 | 诊断用模型 base_link → link6；脚本明写不是标定夹爪 TCP |
| 夹爪 | joint7、joint8 锁定并检查不动，然后仅导出六轴 |
| 输出 | `ROOT/artifacts/visual_demo_trajectory.json`，并不是脚本直接写到当前目录的 trajectory.json |
| 真机 | 默认流程仅可视化；--move-to-start 调用另一个 adapter，脚本注释说明未验收时拒绝执行 |

因此还需从 cuRobo 主机保留/导出实际的 `arx_r5/common.py`、`planning/model.py`、`control/sdk.py`、`trajectory.py`、robot.yml、offline_profile.json 和 r5_offline.urdf；这些实现可能含有额外约定，本地单文件不能核对其内部内容。

## 已找到的 SDK 证据

1. Python SingleArm(type=0) 加载 `X5liteaa0.urdf`，type 非 0 则加载 `R5_master.urdf`。必须确认现场实际 type，不能因机械臂名叫 R5 就自动选择某个模型。
2. `SingleArm.get_joint_positions()` 原样返回底层数组；`set_joint_positions()` 原样传入数组，再切换状态 5。`get_joint_names()` 没有实现。因此 Python 包装层没有显式名称重排、符号反转或零位偏置代码。
3. ROS2 `R5Visualization.cpp` 把 SDK 状态数组直接发布为 joint1…joint7；`R5Controller.cpp` 把前六项直接送入 setJointPositions，第七项用于夹爪。这支持“SDK 前六项对应 joint1…joint6”的候选顺序，对应的是该 SDK/ROS 模型，不能独立证明 R5a 与现场硬件零点完全一致。
4. SDK 笛卡尔接口数组是 `[x,y,z,qw,qx,qy,qz]`；转换至 scipy 默认四元数接口时应重排为 `[qx,qy,qz,qw]`。姿态转换使用弧度。
5. 既有 R5_SDK_FRAME_CHECK.md 记录共享库静态分析：求解链 base_link → link6，SDK FK 还处理相对初始平移。该条是已有审计记录，本轮未重做反汇编，不能据此直接填写数值基座变换。

## 为什么同名 link6 仍有差异

对本地两份 URDF 逐关节比较：

| joint | SDK X5liteaa0 origin xyz (m) | R5a origin xyz (m) |
|---|---|---|
| joint4 | 0.245, 0, -0.06 | 0.245, -0.00005, -0.06 |
| joint5 | 0.06775, -0.00005, -0.0865 | 0.073914, 0.00005, -0.083391 |
| joint6 | 0.02895, 0, 0.0865 | 0.025286, 0, 0.083391 |

joint1…joint6 的局部 axis 相同，分别 Z、Y、Y、Y、Z、X；joint3 origin 的 roll 一份为 +3.1416，一份为 -3.1416（接近同一个半周，但有限精度不完全相等）。这是 origin 旋转，不是“joint3 角度必须反号”的证据。

joint5/6 是实际腕部几何差异；不能未经多姿态验证就把它们当作单一固定的基座/TCP 平移修正。规划主机的 r5_offline.urdf 也必须与这里 R5a 再比对。

## 需要的映射与验证方法

约定 B=规划模型基座、S=SDK 笛卡尔参考系、L=规划 link6、E=SDK 控制末端：

`q_model[i] = sign[i] * q_sdk[index[i]] + offset_rad[i]`

`B_T_L_target = B_T_S @ S_T_E_target @ E_T_L`

逆向导出关节时：`q_sdk[index[i]] = (q_model[i] - offset_rad[i]) / sign[i]`。

当前可提供候选 index=[0,1,2,3,4,5]；全 +1 与零 offset 仅为待验证假设。B_T_S、E_T_L 数值尚无充分证据，保留 null，不能用单位阵冒充已确认结果。

在现场已有 SDK 会话中，读取同一静止姿态的 `get_joint_positions()` 和 `get_ee_pose()`，附 host 时间戳、SDK/URDF 文件 SHA256、type、机械臂版本、启动/回零方式。保存多个不同关节构型的数据；不需要为采样脚本额外发送任何运动命令。顺序、正方向和零点还需要与实际关节/厂家零位对应核对。注意 SDK 返回的第七项是夹爪，不可照搬成 R5a 两个米制 prismatic 关节。

cuRobo 主机用同一组 SDK q 做候选映射后的模型 FK，与同姿态的 SDK S_T_E 比较，估计/验证上述固定变换；保留未用于拟合的姿态做验证，并报告每个姿态位置/旋转残差及最大值。若残差随构型变化，优先检查几何模型差异、关节零点/符号、SDK 初始参考处理；不能靠一个零位样本宣布映射完成。若启动后参考系改变，还应检查跨重启一致性。

本地 trajectory.json 已记录 hardware_eligible=false，hardware_blockers 含 SDK 关节方向/零点/顺序未测量；已有轨迹文件不提供新的 SDK 配对反馈样本。

## 给另一台主机 Codex 的任务文本

请读取这个证据包并定位你所说的 --mapping 的真实 schema。先核对实际加载的 arx_r5 包、r5_offline.urdf、robot.yml、offline_profile.json 与本包模型的区别。把 ROS2 直接数组映射作为关节顺序的来源证据；不要把候选正号、零偏置或 null 坐标变换标为 verified。若只运行 planning_demo.py 的模型关节离线演示，不应要求 SDK 坐标映射；若接入 online_planning.py 的 sdk_reference_T_ee_target，则必须显式处理映射并在 SDK 参考系中做末端 FK 验证。请返回 mapping schema、实际模型指纹和缺少的现场样本字段，使 SDK 主机可据此输出最终配置。

## 官方来源及本包证据边界

- https://github.com/ARXroboticsX/R5 （本轮联网确认仓库可访问）
- https://github.com/ARXroboticsX/R5/blob/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/py/ARX_R5_python/bimanual/script/single_arm.py
- https://github.com/ARXroboticsX/R5/blob/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/ROS2/R5_ws/src/ARX_R5_ros2_V7/arx_r5_controller/src/R5Visualization.cpp
- https://github.com/ARXroboticsX/ARX_Model/blob/74386441845f12af4da203843b6400951d6c5530/R5/R5a/urdf/R5a.urdf

本轮固定提交文件的网页抓取失败；具体结论依据随包的本地文件核对，不声称本轮重新验证其与远端提交逐字节相同。manifest.json 为本地证据 SHA256。single_arm.py 在工作区已有修改，随包保留其当前版本，不覆盖原文件。
