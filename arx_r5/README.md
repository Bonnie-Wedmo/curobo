# ARX R5 离线规划与控制适配

当前使用 R5a 原始运动学与九个 STL、cuRobo GPU 规划、独立轨迹复核及原生
`ViserVisualizer` 回放。代码独立位于本目录，未修改 cuRobo 核心或 UR10e 学习代码。
**当前模型和轨迹只用于离线研究，不能直接驱动真机。**

## 本次修正

旧示例的 `joint3` 反向折叠会导致 `link2/link3` 躯体干涉，此前整对排除相邻连杆漏掉了它。
现在保留原始 URDF 坐标和转轴，新增两组肘部躯体球别名，同时纳入 GPU 和 CPU 碰撞检查。
碰撞配置升级为 `arx-r5-offline-v2-elbow-body-guard`；v1 轨迹会因版本/哈希不同被拒绝，
旧报告中的“通过”不能作为当前验收依据。

最初提供的固定底座参考例让肘、腕反向配合，展示近似保持夹爪朝向的侧向接近：

```python
q_start = [0.0, 0.0, 1.10, -1.00, 0.13, 0.06]
q_goal  = [0.0, 0.0, 0.45, -0.35, 0.13, 0.06]
```

该参考例实测：141 点、dt=0.025 s、3.5 秒；link6 平移约 16.1 cm，最大朝向漂移约 0.00124°。
关节空间轨迹的末端距两端直线最多约 13.2 mm，**不是受硬约束的笛卡尔直线接近**。
link6 是腕端 URDF 坐标系，不是已标定的夹爪 TCP。

源码的 START/GOAL 可自行修改，以当前代码和 JSON 中的实际状态为准。
例如目标改为 `[0.2, 0.02, 0.45, -0.35, 0.11, 0.04]` 后，最新实测位移约 16.4 cm、
朝向漂移约 12.6°；改变关节目标后，固定朝向的参考指标不再适用。

## 运行与审查

```bash
cd /home/mimi/ssy/curobo2/curobo
source ../.venv/bin/activate
python -m arx_r5.planning.demo --port 8086
```

与 UR10e 示例相同，`demo.py` 按 Step 1–7 显式初始化规划器、设置起终点、规划、提取轨迹、
初始化原生 `ViserVisualizer`、显示起点、逐帧调用 `set_joint_state()`。
打开终端打印的 HTTP 地址，再回终端按 Enter 播放；`--autoplay` 跳过等待，
`--speed 0.5` 设置速度，`--spheres` 显示原生碰撞球，Ctrl+C 退出。
每次运行都会实际调用 GPU，并覆盖导出 `artifacts/visual_demo_trajectory.json`。

直接查看 [参考起点](artifacts/approach_start.png)、[固定底座参考终点](artifacts/approach_goal.png)、
[最近一次导出的轨迹](artifacts/visual_demo_trajectory.json)、[肘部网格核查数据](artifacts/elbow_mesh_audit.json)。

每次成功导出会覆盖同名 JSON；规划失败会保留先前文件。因此文件存在不等于这次规划成功，
需要核对终端结果、`created_at_utc`、`start_state`、`goal_state` 和配置版本。

## 验证轨迹与 Mock 执行

先运行上面的示例生成 v2 轨迹，再执行：

```bash
python -m unittest discover -s arx_r5/tests -v
python -m arx_r5.planning.verify_fk
python -S -m arx_r5.trajectory arx_r5/artifacts/visual_demo_trajectory.json
python -m arx_r5.control.run arx_r5/artifacts/visual_demo_trajectory.json \
  --log arx_r5/artifacts/approach_mock.jsonl
MPLCONFIGDIR=/tmp/arx-r5-mpl python -m arx_r5.planning.visualize \
  --trajectory arx_r5/artifacts/visual_demo_trajectory.json \
  --output-dir arx_r5/artifacts/approach_plots
```

无 GPU 的已有 JSON 回放使用独立 CPU 查看器：

```bash
python -m arx_r5.planning.visualize \
  --trajectory arx_r5/artifacts/visual_demo_trajectory.json --viewer
```

需要直接批量规划时，`planning.plan` 仍提供 `--start/--goal/--output` 接口，
或通过 `--start-snapshot` 接收带来源的起点快照。

## 跨机部署准备

```bash
python -m arx_r5.control.state mock \
  --position 0 0 1.1 -1.0 0.13 0.06 \
  --output arx_r5/artifacts/mock_start_snapshot.json
python -m arx_r5.control.preflight arx_r5/artifacts/visual_demo_trajectory.json \
  --current-state arx_r5/artifacts/mock_start_snapshot.json \
  --output arx_r5/artifacts/deployment_preflight_v2.json
```

空现场模板下，预检应写出受阻报告并退出 2；不会连接 SDK。
真实关节映射、厂家动态限制、实际几何、控制时序及可靠停止仍需现场验收。

## 文档导航

- [可视化与逐步审查](docs/howto/visualization.md)
- [模型设计、肘部碰撞与关节目标的边界](docs/explanation/design.md)
- [官方资源及参数来源](docs/reference/resources.md)
- [轨迹与代码接口](docs/reference/interface.md)
- [状态、标定与预检字段](docs/reference/deployment_interface.md)
- [跨计算机部署流程](docs/howto/deployment.md)
- [v1 历史结果](docs/reference/results.md)、[v1 历史运动矩阵](docs/reference/deployment_results.md)

`artifacts/` 中早期 `trajectory.json`、`deployment_suite/` 和旧 GIF/截图保留为历史记录；
它们使用 v1 碰撞策略，尤其不能再据此认定相邻肘部无干涉。当前演示以 v2 新轨迹和新图为准。
