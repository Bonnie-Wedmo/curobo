import torch

from curobo.inverse_kinematics import (
    InverseKinematics,
    InverseKinematicsCfg,
)

from curobo.types import JointState, GoalToolPose


# Step 1: 配置 IK 求解器
n_poses = 16

config = InverseKinematicsCfg.create(
    robot="ur10e.yml",
    num_seeds=8,
    max_batch_size=n_poses,
    self_collision_check=False,
)

ik = InverseKinematics(config)

target_link = ik.tool_frames[0]


# Step 2: 生成多组目标位姿
torch.manual_seed(42)

q_ref = ik.default_joint_state.position

q_batch = q_ref.unsqueeze(0).repeat(n_poses, 1)

# 对关节角施加随机扰动
q_batch += (torch.rand_like(q_batch) - 0.5) * 0.4

# 使用 FK 生成已知运动学可达的目标
joint_states = JointState.from_position(
    q_batch,
    joint_names=ik.joint_names,
)

fk_state = ik.compute_kinematics(joint_states)

target_poses = fk_state.tool_poses.get_link_pose(target_link)


# Step 3: 构造批量 IK 输入
goals = GoalToolPose.from_poses(
    {target_link: target_poses},
    num_goalset=1,
)


# Step 4: 批量 IK 求解
result = ik.solve_pose(goals)


# Step 5: 输出结果
print("\n=== Batched IK ===")

print("Number of targets:", n_poses)

print("Success shape:", result.success.shape)

n_success = result.success.sum().item()

print("Successful solutions:", n_success)

print("Success rate:", n_success / n_poses * 100, "%")

if n_success > 0:

    success_mask = result.success.bool()

    pos_errors = result.position_error[success_mask]

    print(
        "Mean position error:",
        pos_errors.mean().item() * 1000,
        "mm"
    )

    print(
        "Max position error:",
        pos_errors.max().item() * 1000,
        "mm"
    )