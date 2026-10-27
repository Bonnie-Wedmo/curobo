# 按 UR10e 的代码流程显示 R5 运动

当前为 v2 侧向接近示例：已修正旧负肘角的躯体漏检。此前截图与 GIF 作为 v1 历史保留，
不能再用于证明旧姿态无碰撞。

目标：在本机查看经过规划和检查的明显位移轨迹，并逐点对照模型、末端路径和起终点。

## 直接播放

从 cuRobo 仓库根目录运行：

```bash
source ../.venv/bin/activate
python -m arx_r5.planning.demo
```

打开终端打印的 HTTP 地址，保持 Python 进程运行。默认先显示起点，返回终端按 Enter 后
以原轨迹的 0.5 倍速度循环回放；每轮结束暂停一秒后重新显示起点，并未规划返回轨迹。

需要指定一个新端口，避免打开仍运行旧代码的页面：

```bash
python -m arx_r5.planning.demo --port 8086
```

需要跳过终端等待直接播放，或改变回放速度：

```bash
python -m arx_r5.planning.demo --port 8086 --autoplay --speed 0.8
```

此入口现在每次实际进行 GPU 规划，并使用 cuRobo 原生 `ViserVisualizer`，需要现有 GPU 环境。
原先的 `--replan`、`--paused` 参数已移除。独立的无 GPU JSON 查看方式见后文。
默认运行不连接 SDK。

## 真机移到 START 的准备入口（尚未启用执行）

显式请求这一准备步骤：

```bash
python -m arx_r5.planning.demo --move-to-start
```

`demo.py` 的 Step 0 调用 `OfficialR5Adapter().move_to_start(START)`。
当前官方示例尚未在控制电脑跑通，此命令在 SDK 构造前拒绝，退出码为 2；
不会移动机械臂，也不会继续启动离线回放。此入口是待接入位置，不是已完成的真机功能。

后续接入须完成“读取并映射真实当前状态 → 规划和检查 current 到 START 的轨迹 →
按已验证时序执行 → 用反馈确认到位”，再继续现有 demo。
[官方 SingleArm 源码](https://github.com/ARXroboticsX/R5/blob/master/py/ARX_R5_python/bimanual/script/single_arm.py)
的 `set_joint_positions()` 只下发目标并调用 `set_arm_status(5)`，没有等待到位逻辑。
`go_home()` 使用状态 1，目标是厂商预设回零姿态，不能替代这里自定义的 `START`。

## 对照 UR10e 审查代码

直接打开 [demo.py](../../planning/demo.py)，按照注释 Step 1–7 检查：

1. `MotionPlannerCfg.create()` → `MotionPlanner()`，终端输出实际 URDF 和网格根目录。
2. 用两个六轴 `JointState` 指定起点和目标。
3. `planner.plan_cspace()` 执行关节空间规划。失败立即中止，不初始化空场景。
4. `result.get_interpolated_plan()`，先检查两个锁定夹爪，再提取主动六轴；终端输出原始和回放形状。
   JSON 导出仍保留数值、导数与独立自碰撞检查。
5. `ViserVisualizer(content_path=ContentPath(...), add_robot_to_scene=True, ...)`，
   与 UR10e 使用同一类、同一原生 URDF 加载链；终端输出网格数量和可视化关节名称。
6. `viewer.set_joint_state(initial_state)` 设置起点，`input()` 等待 Enter。
7. `for i in range(n_steps)` 中构造 `JointState`，调用 `viewer.set_joint_state()`，
   用 `time.sleep(dt / playback_speed)` 控制回放。Ctrl+C 停止服务。

主流程与 UR10e 的差别是 R5 使用 `plan_cspace`、显式外部模型路径及额外的夹爪检查。
新 JSON 的 `task_space_diagnostics` 记录实际末端位移、朝向漂移和直线偏差。
JSON 文件查看器 `visualize.py` 继续独立保留，不再是 `demo.py` 的可视化实现。

需要检查原生碰撞球时执行：

```bash
python -m arx_r5.planning.demo --port 8086 --spheres
```

默认关闭碰撞球，避免遮住网格。蓝色路径连接实际采样点的 `link6` 原点，不是夹爪尖端路径。
网页中的末端控制框只用于显示，不会改变离线关节轨迹或下发硬件指令。

固定底座参考起终点（`joint1` 至 `joint6`，单位 rad；修改源码后以实际 JSON 为准）：

```python
q_start = [0.0, 0.0, 1.10, -1.00, 0.13, 0.06]
q_goal  = [0.0, 0.0, 0.45, -0.35, 0.13, 0.06]
```

本机实测得到 141 点、dt=0.025 s、3.5 秒轨迹；link6 原点两端距离约 0.16108 m，
途中最大朝向漂移约 0.00124°。关节 1 不转，肘、腕反向配合，适合展示保持朝向的侧向接近。
这仍是关节空间规划；末端偏离两端直线最多约 13.2 mm，不能当作抓取末段的直线接近约束。
独立检查加密为 219 个姿态，最小球间隙约 0.117 mm，仍不能覆盖真实装配、标定和跟踪误差。

上述数值只针对固定底座参考状态；改变 `GOAL` 后，朝向漂移可能显著增加。
每次规划以新 JSON 的 `task_space_diagnostics` 为准，不能沿用参考数值。

## 无 GPU 的 JSON 文件查看

```bash
python -m arx_r5.planning.visualize \
  --trajectory arx_r5/artifacts/visual_demo_trajectory.json --viewer --port 8084
```

这条命令使用原先的 CPU 场景回放，含播放、滑块和起终点轮廓，适合检查现有文件；
它与本页前半部分的原生 GPU 演示是两个明确独立的入口。

导出六轴位置、速度、加速度曲线和模型球体静态图：

```bash
MPLCONFIGDIR=/tmp/arx-r5-mpl python -m arx_r5.planning.visualize \
  --trajectory arx_r5/artifacts/visual_demo_trajectory.json \
  --output-dir arx_r5/artifacts/visual_demo
```

无浏览器服务时，可直接打开新示例的 [起点](../../artifacts/approach_start.png)、
[终点](../../artifacts/approach_goal.png) 截图。

## 空白或无法连接时

1. 核对终端地址。Viser 使用 `http://` 与 `ws://`；本程序没有配置 HTTPS/TLS。
   端口占用时 Viser 可能选择后续端口，以实际打印地址为准。
2. 确认进程仍在运行。`--smoke-seconds` 会自动结束服务，日常回放不要添加它。
3. 先检查终端停在哪一个编号步骤。尚未到 Step 5 表示 Viser 未创建；到 Step 6 并打印
   `Loaded visual meshes: 9` 表示模型已加载。Enter 只影响播放，不应影响起点模型显示。
4. 整页完全空白时，用 F12 → Console 查看首条红色错误，用 Network 检查页面脚本及 WebSocket。
   用户反馈的同机 `http://127.0.0.1:8082/` 在本次检查中返回 HTTP 200，且软件 WebGL 的
   Chromium 能显示场景，用户随后确认问题已解决。未确定唯一根因；今后若再次发生没有控件的
   整页空白，可先检查脚本和 WebGL，不应直接判断为 STL 导入失败。
5. 如果浏览器运行在另一台电脑，`127.0.0.1` 指向浏览器所在电脑。可使用 SSH 本地端口转发，
   或在确认网络访问范围后用 `--host` 指定服务绑定地址。

本次只验证本机 HTTP 页面；外部 HTTPS 反向代理、端口转发及特定浏览器插件未作验收。
