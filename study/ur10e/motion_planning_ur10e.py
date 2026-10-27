import torch
import time

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose
from curobo.viewer import ViserVisualizer
from curobo.types import ContentPath, JointState

# Step 1: 配置 Motion Planner
config = MotionPlannerCfg.create(
    robot="ur10e.yml",
    scene_model=None,
    self_collision_check=True,
    num_ik_seeds=8,
    num_trajopt_seeds=2,
    use_cuda_graph=False,
)

planner = MotionPlanner(config)

print("Robot joints:", planner.joint_names)
print("Tool frames:", planner.tool_frames)


# Step 2: 设置起始关节状态
q_start = planner.default_joint_state.clone().unsqueeze(0)

print("\nStart joints:", q_start.position)


# Step 3: 构造已知可达的目标位姿
q_goal = q_start.clone()

q_goal.position[0, 0] += 0.25
q_goal.position[0, 2] -= 0.20

goal_kinematics = planner.compute_kinematics(q_goal)

target_link = planner.tool_frames[0]

target_pose = goal_kinematics.tool_poses.get_link_pose(
    target_link
)

goal = GoalToolPose.from_poses(
    {target_link: target_pose},
    num_goalset=1,
)

print("\nTarget position:", target_pose.position)
print("Target quaternion:", target_pose.quaternion)


# Step 4: 运动规划
print("\nPlanning...")

result = planner.plan_pose(
    goal_tool_poses=goal,
    current_state=q_start,
)


# Step 5: 检查规划结果
print("\n=== Motion Planning Result ===")

if result is not None and result.success.any().item():
    print("Planning succeeded!")
    trajectory = result.get_interpolated_plan()

    # ==========================================
    # Step 7: UR10e 3D Visualization
    # ==========================================

    print("\n=== Visualization Debug ===")
    print("Trajectory shape:", trajectory.position.shape)
    print("Trajectory joint names:", trajectory.joint_names)
    print("Planner joint names:", planner.joint_names)
    print("Planner DOF:", len(planner.joint_names))
    print("Before Viser initialization", flush=True)

    # 1. 初始化 Viser
    viewer = ViserVisualizer(
        content_path=ContentPath(
            robot_config_file="ur10e.yml"
        ),
        connect_ip="127.0.0.1",
        connect_port=8080,
        add_robot_to_scene=True,
        add_control_frames=True,
        visualize_robot_spheres=True,
    )

    # 2. 获取规划轨迹
    joint_trajectory = trajectory.position[0, 0, :, :].detach()

    n_steps = joint_trajectory.shape[0]

    dt = float(
        planner.trajopt_solver.config.interpolation_dt
    )

    print("\n=== 3D Visualization ===")
    print("Trajectory shape:", joint_trajectory.shape)
    print("Number of frames:", n_steps)
    print("Time interval:", dt)

    # 3. 将机器人初始化到轨迹起点
    initial_state = JointState.from_position(
        joint_trajectory[0],
        joint_names=planner.joint_names,
    )

    viewer.set_joint_state(initial_state)

    # 4. 打开浏览器
    print("\nOpen: http://localhost:8080")
    input("Open the viewer, then press Enter to play...")

    # 5. 循环播放规划轨迹
    playback_speed = 0.5

    try:
        while True:

            for i in range(n_steps):

                # 获取当前轨迹点
                joint_position = joint_trajectory[i]

                # 构造 JointState
                joint_state = JointState.from_position(
                    joint_position,
                    joint_names=planner.joint_names,
                )

                # 更新 UR10e 三维模型
                viewer.set_joint_state(joint_state)

                # 控制动画播放速度
                time.sleep(dt / playback_speed)

            # 每次播放结束暂停 1 秒
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nVisualization stopped.")


    print("Trajectory shape:", trajectory.position.shape)
    print("First waypoint:", trajectory.position[0, 0, :])
    print("Last waypoint:", trajectory.position[0, -1, :])
    print("Planning time:", result.total_time, "s")
    print("Position error:", result.position_error)

else:

    print("Planning failed!")