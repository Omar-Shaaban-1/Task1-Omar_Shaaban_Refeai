"""
collision_checker.py
---------------------
Validates a trajectory BEFORE it is sent to the hardware/simulator.

Mirrors the slide "Collision Avoidance and Scene Monitoring":
  - State Validity: each joint's position vs. hard joint limits
    and static obstacles.
  - Self-Collision: simplified here as a minimum-distance check
    between non-adjacent links (treating each link as a sphere
    around its joint position — a coarse but fast approximation
    of what FCL does with real mesh geometry).
  - Dynamic Interception: placeholder hook for live sensor updates
    aborting a trajectory mid-motion.

In a full ROS deployment you would replace this with MoveIt's
planning scene + FCL. This standalone version lets you test the
LOGIC of "does this path pass or fail" without needing MoveIt
installed.
"""

import numpy as np
from fk_solver import forward_kinematics, DEFAULT_DH_TABLE


class Obstacle:
    """A simple spherical obstacle in the workspace."""

    def __init__(self, center, radius):
        self.center = np.array(center, dtype=float)
        self.radius = radius


def check_joint_limits(joint_angles, joint_limits):
    for i, (lo, hi) in enumerate(joint_limits):
        if not (lo <= joint_angles[i] <= hi):
            return False, f"Joint {i} out of limits: {joint_angles[i]:.3f} not in [{lo:.3f}, {hi:.3f}]"
    return True, ""


def check_self_collision(joint_positions, min_link_separation=0.05):
    """
    Coarse self-collision check: flag if any two NON-ADJACENT
    joint positions get closer than min_link_separation.
    """
    n = len(joint_positions)
    for i in range(n):
        for j in range(i + 2, n):  # skip adjacent joints (they're always close)
            dist = np.linalg.norm(joint_positions[i] - joint_positions[j])
            if dist < min_link_separation:
                return False, f"Self-collision risk between joint {i} and joint {j} (dist={dist:.3f}m)"
    return True, ""


def check_obstacle_collision(joint_positions, obstacles, link_radius=0.04):
    """Check each link point against every obstacle sphere."""
    for obs in obstacles:
        for idx, p in enumerate(joint_positions):
            dist = np.linalg.norm(p - obs.center)
            if dist < (obs.radius + link_radius):
                return False, f"Collision with obstacle at joint {idx} (dist={dist:.3f}m, threshold={obs.radius + link_radius:.3f}m)"
    return True, ""


def validate_trajectory(trajectory, joint_limits=None, obstacles=None,
                         dh_table=DEFAULT_DH_TABLE):
    """
    Run every waypoint of a planned trajectory through the full
    validity pipeline. Returns on the FIRST failure (like the
    "TRAJECTORY ABORT" behavior shown in the slides).

    Args:
        trajectory: dict from trajectory_planner.generate_joint_trajectory
        joint_limits: list of (min, max) per joint, radians
        obstacles: list of Obstacle objects

    Returns:
        (valid: bool, message: str, failed_index: int or None)
    """
    n_joints = trajectory["positions"].shape[1]
    if joint_limits is None:
        joint_limits = [(-np.pi, np.pi)] * n_joints
    if obstacles is None:
        obstacles = []

    for i, q in enumerate(trajectory["positions"]):
        ok, msg = check_joint_limits(q, joint_limits)
        if not ok:
            return False, f"[t={trajectory['times'][i]:.2f}s] {msg}", i

        _, joint_positions = forward_kinematics(q, dh_table)

        ok, msg = check_self_collision(joint_positions)
        if not ok:
            return False, f"[t={trajectory['times'][i]:.2f}s] {msg}", i

        ok, msg = check_obstacle_collision(joint_positions, obstacles)
        if not ok:
            return False, f"[t={trajectory['times'][i]:.2f}s] {msg}", i

    return True, "Trajectory clear — no collisions detected.", None


if __name__ == "__main__":
    from trajectory_planner import generate_joint_trajectory

    q_start = np.zeros(6)
    q_goal = np.deg2rad([30, -20, 45, 0, 60, -10])
    traj = generate_joint_trajectory(q_start, q_goal, duration=2.5, n_waypoints=20)

    # Case 1: empty workspace -> should pass
    valid, msg, idx = validate_trajectory(traj)
    print("No obstacles:", valid, "-", msg)

    # Case 2: put an obstacle right in the arm's path -> should fail
    blocking_obstacle = Obstacle(center=[0.15, 0.0, 0.35], radius=0.15)
    valid, msg, idx = validate_trajectory(traj, obstacles=[blocking_obstacle])
    print("With obstacle:", valid, "-", msg)
