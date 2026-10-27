import torch

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import JointState


# Step 1: 加载 UR10e 模型
config = KinematicsCfg.from_robot_yaml_file("ur10e.yml")
robot = Kinematics(config)

# Step 2: 构造关节角输入
q = torch.tensor(
    [[0.0, -2.2, 1.9, -1.383, -1.57, 0.0]],
    device="cuda",
    dtype=torch.float32,
)

joint_state = JointState.from_position(
    q,
    joint_names=robot.joint_names,
)


# Step 3: 正运动学计算
state = robot.compute_kinematics(joint_state)


# Step 4: 获取末端位姿
ee_pose = state.tool_poses.get_link_pose("tool0")

print("\n=== Forward Kinematics ===")
print("Joint configuration:", q)
print("Joint configuration shape:", q.shape)

print("\nEnd-effector position:")
print(ee_pose.position)

print("\nEnd-effector quaternion (wxyz):")
print(ee_pose.quaternion)

# Step 5: Batched FK

batch_size = 1000

# 以 UR10e 默认关节配置为基准
q_default = q.clone()

# 生成 1000 组关节配置
q_batch = q_default.repeat(batch_size, 1)

# 在默认关节角附近施加小扰动
q_batch += (torch.rand_like(q_batch) - 0.5) * 0.2

# GPU 预热
joint_state_batch = JointState.from_position(
    q_batch,
    joint_names=robot.joint_names,
)

robot.compute_kinematics(joint_state_batch)
torch.cuda.synchronize()

# GPU 计时
start = torch.cuda.Event(enable_timing=True)
end = torch.cuda.Event(enable_timing=True)

start.record()

state_batch = robot.compute_kinematics(joint_state_batch)

end.record()
torch.cuda.synchronize()

# 输出
print("\n=== Batched FK ===")
print("Input shape:", q_batch.shape)
print("Output shape:", state_batch.tool_poses.position.shape)
print("GPU execution time:", start.elapsed_time(end), "ms")

# Step 6: Differentiable FK

# 创建需要计算梯度的关节角
q_grad = q.clone().detach().requires_grad_(True)

# FK
joint_state_grad = JointState.from_position(
    q_grad,
    joint_names=robot.joint_names,
)

state_grad = robot.compute_kinematics(joint_state_grad)

ee_pos = state_grad.tool_poses.get_link_pose("tool0").position

# 设置目标位置：当前末端位置沿 x 方向移动 5 cm
target_pos = ee_pos.detach() + torch.tensor(
    [[0.05, 0.0, 0.0]],
    device="cuda",
    dtype=torch.float32,
)

# 定义损失函数
loss = torch.sum((ee_pos - target_pos) ** 2)

# 反向传播
loss.backward()

# 输出
print("\n=== Differentiable FK ===")
print("Loss:", loss.item())
print("Joint gradients:", q_grad.grad)
print("Gradient shape:", q_grad.grad.shape)