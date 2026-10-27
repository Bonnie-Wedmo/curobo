# R5 SDK 末端坐标核对

检查对象：ARXroboticsX/R5 提交 `e87d09c11edb65f6ae672abc7a41fe2277a2f12f` 中的
Python 接口、模型与 x86/ARM64 共享库。仅静态读取 ELF、符号和指令，未加载或执行机械臂 SDK。

## 证据

1. `single_arm.py` 将受控目标 `[x,y,z,qw,qx,qy,qz]` 原样传给 `InterfacesPy::set_ee_pose`。
2. x86 的 `ControllerBase` 构造函数在 `0x151308` 引用 `link6`、`0x151334` 引用 `base_link`，
   随后 `0x15135e` 调用带根/末端名称的 `KinematicDynamicSolver` 构造函数。
3. ARM64 的对应构造函数在 `0x13dddc` 构造 `base_link`、`0x13de00` 构造 `link6`，
   随后 `0x13de24` 调用同一签名的运动学构造函数。
4. `ControllerBase::loadJointStatus` 调用该求解器的 FK；`InterfacesPy::get_ee_pose` 从
   `ControllerThread::getEndPose` 取得反馈矩阵并提取其平移和四元数。
   x86 调用位置 `0x1638a6`，ARM64 调用位置 `0x14f264`。
5. `set_ee_pose` 将传入的位置和四元数组成矩阵后交给 `ControllerThread::setEndPose`，
   接口层未再附加指腹 TCP 平移。求解链的受控末端为 link6。

SDK 的 FK 路径还包含相对初始平移的处理，因此本项目继续使用配对 get/set API 的 SDK
笛卡尔参考系 S，不把反馈数值直接称为 URDF 的 `base_link_T_link6`。
这不改变末端局部坐标系为 link6，也不影响在同一 SDK 参考系内完成相机与工具变换。

本地受检文件的 Git blob SHA1 与该提交一致：

| 文件 | Git blob SHA1 |
|---|---|
| `libarx_r5_src.so` | `133093922eaf55e8e40fac63b5be19a116150097` |
| `libarx_r5_src-arm64.so` | `138e88587485fb7e7be9613b688391b4d5b2ee49` |

## 当前采用的定义

- 根据用户对标定末端为 link6 的说明，标定参考系 W 采用 link6，E_T_W 为单位矩阵。
- 根据用户最后确认的数值，抓取参考点为 link6 原点沿其 +X 平移 **0.15 m**，XYZ 轴继承 link6。
- E_T_G 的平移就是 `(0.15,0,0)`；不再叠加夹指原点、指腹网格中心或此前的 3 cm。
- 此为用户指定的工具参考点，不声称是制造商给定的 TCP 或实测指腹面积中心。
- 已有 C_T_G_pre 包含的 20 cm 预抓取后退仍直接读取，不重新计算。

共享库核对不能证明用户私有标定程序的实现；这里的 W=link6 来自用户给出的标定定义。
若实际部署更换 SDK 二进制、工具或标定参考点，需要重新核对相应变换。
当前 SDK 使用的 X5liteaa0.urdf 与 ARX_Model 的 R5a 几何有差异，本次没有替换 SDK 模型。

## 官方文件

- [Python 接口](https://github.com/ARXroboticsX/R5/blob/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/py/ARX_R5_python/bimanual/script/single_arm.py)
- [SDK 共享库目录](https://github.com/ARXroboticsX/R5/tree/e87d09c11edb65f6ae672abc7a41fe2277a2f12f/py/ARX_R5_python/bimanual/lib/arx_r5_src)
- [R5a 模型](https://github.com/ARXroboticsX/ARX_Model/blob/74386441845f12af4da203843b6400951d6c5530/R5/R5a/urdf/R5a.urdf)
