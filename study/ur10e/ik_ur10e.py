from curobo.inverse_kinematics import (
    InverseKinematics,
    InverseKinematicsCfg,
)

from curobo.types import GoalToolPose


# Step 1: 创建 IK 配置
config = InverseKinematicsCfg.create(
    robot="ur10e.yml",
    num_seeds=32,
    self_collision_check=False,
)

# Step 2: 初始化 IK 求解器
ik = InverseKinematics(config)

target_link = ik.tool_frames[0]

print("Target link:", target_link)


# Step 3: 使用默认关节配置，通过 FK 生成目标位姿
q_ref = ik.default_joint_state.clone()

fk_state = ik.compute_kinematics(q_ref)

target_pose = fk_state.tool_poses.get_link_pose(target_link)

print("\n=== Target Pose ===")
print("Reference joints:", q_ref.position)
print("Target position:", target_pose.position)
print("Target quaternion:", target_pose.quaternion)


# Step 4: 将目标位姿封装成 IK 输入
goal = GoalToolPose.from_poses(
    {target_link: target_pose},
    num_goalset=1,
)


# Step 5: 求解 IK
result = ik.solve_pose(goal)


# Step 6: 检查求解结果
print("\n=== IK Result ===")
print("Success:", result.success)

if result.success.any().item():

    print("Solution:", result.js_solution.position)

    print(
        "Position error:",
        result.position_error.item() * 1000,
        "mm"
    )

    print(
        "Rotation error:",
        result.rotation_error.item(),
        "rad"
    )

else:
    print("IK failed!")