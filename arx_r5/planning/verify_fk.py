"""cuRobo GPU FK 对照独立 URDF 变换链和 yourdfpy；不代表实机标定。"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from arx_r5.common import JOINT_NAMES, ROOT, load_profile, write_json
from arx_r5.planning.model import forward_kinematics, robot_config


def verify() -> dict:
    """测试零位、单关节正负扰动及非零关节组合，检查所有 link 位姿。"""
    import torch
    import yourdfpy
    from curobo.kinematics import Kinematics, KinematicsCfg
    from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
    from curobo.types import JointState

    cfg = robot_config()
    links = cfg["robot_cfg"]["kinematics"]["mesh_link_names"]
    kin = Kinematics(KinematicsCfg.from_robot_yaml_file(cfg, tool_frames=links))
    if kin.joint_names != JOINT_NAMES:
        raise ValueError("cuRobo joint order mismatch")
    reference = yourdfpy.URDF.load(str(ROOT / "models/r5_offline.urdf"), load_meshes=False)
    states = [[0.0] * 6, load_profile()["default_position"]]
    for j in range(6):
        for delta in (-0.2, 0.2):
            q = [0.0] * 6
            q[j] = delta
            states.append(q)
    result = kin.compute_kinematics(JointState.from_position(
        torch.tensor(states, dtype=torch.float32, device="cuda"), joint_names=JOINT_NAMES))
    max_position, max_rotation, max_reference = 0.0, 0.0, 0.0
    for i, q in enumerate(states):
        ref = forward_kinematics(q)
        reference.update_cfg(dict(zip(JOINT_NAMES, q)) | load_profile()["locked_gripper"])
        for link in links:
            pose = result.tool_poses.get_link_pose(link)
            position = pose.position.detach().cpu().numpy().reshape(len(states), 3)[i]
            quaternion = pose.quaternion.detach().cpu().numpy().reshape(len(states), 4)[i]
            rotation = Rotation.from_quat(quaternion[[1, 2, 3, 0]]).as_matrix()
            max_position = max(max_position, float(np.linalg.norm(position - ref[link][:3, 3])))
            max_rotation = max(max_rotation, float(Rotation.from_matrix(
                rotation.T @ ref[link][:3, :3]).magnitude()))
            max_reference = max(max_reference, float(np.abs(
                reference.get_transform(link) - ref[link]).max()))
    if max_position > 2e-5 or max_rotation > 2e-5 or max_reference > 2e-5:
        raise ValueError(f"FK mismatch: {max_position}, {max_rotation}, {max_reference}")
    report = {"passed": True, "states": len(states), "links": links,
              "max_position_error_m": max_position, "max_orientation_error_rad": max_rotation,
              "yourdfpy_max_matrix_error": max_reference,
              "scope": "URDF implementation consistency only; physical zero/direction unverified"}
    # 同时验证 GPU 检查器确实区分安全测试姿态和明确碰撞姿态，而非只看开关。
    checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=cfg))
    collision_states = [load_profile()["default_position"], [0.0, -1.0, 0.6, 0.0, 0.0, 0.0],
                        [-0.8, -3.15, -1.3, -0.42, 0.13, 0.06]]
    _, self_penetration = checker.get_scene_self_collision_distance_from_joints(
        torch.tensor(collision_states, dtype=torch.float32, device="cuda").unsqueeze(1))
    penetration = self_penetration.detach().cpu().reshape(-1).tolist()
    if len(penetration) != 3 or penetration[0] > 0.0 or any(x <= 0.0 for x in penetration[1:]):
        raise ValueError(f"GPU self-collision check failed: {penetration}")
    # 当前 CUDA 核返回半加权的平方距离代价；不能标成以米为单位的间距。
    report["gpu_self_collision_cost"] = penetration
    write_json(ROOT / "artifacts/fk_validation.json", report)
    return report


if __name__ == "__main__":
    print(verify())
