from curobo.kinematics import Kinematics, KinematicsCfg

config = KinematicsCfg.from_robot_yaml_file("ur10e.yml") # 读取 UR10e 机器人配置

robot = Kinematics(config) # 根据配置构建机器人运动学模型

# 检查加载结果
print("Robot model loaded successfully!")
print("DOF:", robot.get_dof())
print("Joint names:", robot.joint_names)
print("Tool frames:", robot.tool_frames)
print("Collision spheres:", robot.total_spheres)