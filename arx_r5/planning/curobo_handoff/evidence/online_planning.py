"""Hardware-free online planner boundary, compatible with test.py trajectory arrays.

The caller supplies a planning backend, limits and tolerances. This module
does not claim that an IK solve or interpolation is a collision-aware planner.
No global solver, SDK object, motor command or default physical limit exists.
"""
import hashlib
import json

import numpy as np

from pregrasp_pose import rigid_transform


def vector6(value, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (6,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain six finite joint values")
    return value


def build_planning_request(start_joints_rad, sdk_reference_T_ee_target, *,
                           capture_stamp, feedback_read_unix_ns,
                           joint_lower_limits_rad, joint_upper_limits_rad,
                           max_joint_velocity_rad_s, scene_context=None):
    """Create a serializable snapshot for a worker/thread/process or remote backend.

    Start joints must be read anew for EACH stage. The timestamp is the host
    feedback-read time, not a claim of hardware-synchronized acquisition.
    """
    start = vector6(start_joints_rad, "start joints")
    lower = vector6(joint_lower_limits_rad, "lower limits")
    upper = vector6(joint_upper_limits_rad, "upper limits")
    speed = vector6(max_joint_velocity_rad_s, "velocity limits")
    if np.any(lower >= upper) or np.any(speed <= 0) or np.any(start < lower) or np.any(start > upper):
        raise ValueError("Invalid limits or start state outside supplied limits")
    if any(not isinstance(v, int) or isinstance(v, bool) or v <= 0
           for v in (capture_stamp, feedback_read_unix_ns)):
        raise ValueError("Require explicit positive capture/feedback timestamps")
    request = {
        "schema_version": 1, "data_kind": "online_motion_request",
        "reference_frame": "sdk_cartesian_reference", "controlled_frame": "sdk_controlled_ee",
        "capture_host_frameset_received_unix_ns": capture_stamp,
        "feedback_read_unix_ns": feedback_read_unix_ns,
        "start_joint_positions_rad": start.tolist(),
        "sdk_reference_T_ee_target": rigid_transform(sdk_reference_T_ee_target).tolist(),
        "joint_lower_limits_rad": lower.tolist(), "joint_upper_limits_rad": upper.tolist(),
        "max_joint_velocity_rad_s": speed.tolist(),
        "scene_context": scene_context,
        "required_backend_checks": ["robot_and_scene_collision", "continuous_path_and_time_parameterization"],
    }
    request["request_id"] = hashlib.sha256(
        json.dumps(request, sort_keys=True, allow_nan=False).encode("utf-8")).hexdigest()
    return request


def plan_online(request, planner_backend, forward_kinematics, *, start_tolerance_rad,
                 position_tolerance_m, angle_tolerance_rad):
    """Call an injected backend and validate its snapshot, joint path and FK goal.

    Backend signature: planner_backend(request) -> dict with request_id,
    time_from_start [seconds] and joint_positions [radians, N x 6].
    FK callback MUST return SDK-reference-to-link6 4x4 for six joint angles.
    Run this function in a caller-owned worker to plan alongside vision.
    A returned trajectory remains unexecuted; these checks alone do not certify
    collision clearance, continuous velocity/acceleration or current-state freshness.
    """
    if (not np.isfinite([start_tolerance_rad, position_tolerance_m, angle_tolerance_rad]).all()
            or start_tolerance_rad <= 0 or position_tolerance_m <= 0
            or not 0 < angle_tolerance_rad <= np.pi):
        raise ValueError("Supply positive start/FK residual tolerances")
    # Give backend a copy so its mutation cannot change our validation snapshot.
    result = planner_backend(json.loads(json.dumps(request)))
    if result.get("request_id") != request["request_id"]:
        raise ValueError("Planner response belongs to a different request")
    times = np.asarray(result.get("time_from_start"), dtype=float)
    joints = np.asarray(result.get("joint_positions"), dtype=float)
    if (times.ndim != 1 or len(times) < 2 or joints.shape != (len(times), 6)
            or not np.isfinite(times).all() or not np.isfinite(joints).all()
            or times[0] != 0 or np.any(np.diff(times) <= 0)):
        raise ValueError("Require increasing times from zero and N x 6 finite joint positions")
    start = vector6(request["start_joint_positions_rad"], "start")
    lower, upper = (vector6(request[key], key) for key in
                    ("joint_lower_limits_rad", "joint_upper_limits_rad"))
    speed = vector6(request["max_joint_velocity_rad_s"], "velocity limits")
    if np.max(np.abs(joints[0] - start)) > start_tolerance_rad:
        raise ValueError("Trajectory starts at a different feedback state")
    if np.any(joints < lower) or np.any(joints > upper):
        raise ValueError("Trajectory violates supplied joint limits")
    if np.any(np.abs(np.diff(joints, axis=0) / np.diff(times)[:, None]) > speed + 1e-12):
        raise ValueError("Trajectory segment average velocity exceeds supplied limits")
    goal = rigid_transform(request["sdk_reference_T_ee_target"])
    actual = rigid_transform(forward_kinematics(joints[-1].copy()))
    position_error = float(np.linalg.norm(actual[:3, 3] - goal[:3, 3]))
    rotation = actual[:3, :3].T @ goal[:3, :3]
    angle_error = float(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
    if position_error > position_tolerance_m or angle_error > angle_tolerance_rad:
        raise ValueError(f"Planner endpoint FK residual failed: {position_error} m, {angle_error} rad")
    return {
        "schema_version": 1, "request_id": request["request_id"],
        "capture_host_frameset_received_unix_ns": request["capture_host_frameset_received_unix_ns"],
        "units": {"position": "rad", "time": "s"},
        "time_from_start": times.tolist(), "joint_positions": joints.tolist(),
        "fk_position_error_m": position_error, "fk_angle_error_rad": angle_error,
        "robot_commands_sent": False,
        "required_before_execution": ["backend_collision_and_continuous_path_checks",
                                      "start_feedback_recheck", "arrival_monitoring"],
    }
