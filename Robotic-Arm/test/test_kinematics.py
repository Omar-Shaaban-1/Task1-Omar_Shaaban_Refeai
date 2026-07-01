"""
test_kinematics.py
--------------------
Run with:  pytest test/test_kinematics.py -v

These tests don't require ROS 2 — just numpy — so you can validate
your math before ever touching Gazebo.
"""

import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "robotic_arm"))

from fk_solver import forward_kinematics, pose_to_xyz
from ik_solver import inverse_kinematics_multi_seed
from trajectory_planner import generate_joint_trajectory
from collision_checker import validate_trajectory, Obstacle


def test_fk_runs_and_returns_valid_shape():
    angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    T, joints = forward_kinematics(angles)
    assert T.shape == (4, 4)
    assert len(joints) == 7  # base + 6 joints


def test_ik_recovers_known_fk_target():
    true_angles = [0.2, 0.5, -0.4, 0.1, 0.3, -0.2]
    T_true, _ = forward_kinematics(true_angles)
    target = pose_to_xyz(T_true)

    success, q_solution, err = inverse_kinematics_multi_seed(target)
    assert success
    assert err < 1e-3

    T_check, _ = forward_kinematics(q_solution)
    recovered = pose_to_xyz(T_check)
    assert np.allclose(recovered, target, atol=1e-3)


def test_trajectory_starts_and_ends_at_rest():
    q_start = np.zeros(6)
    q_goal = np.deg2rad([20, -10, 15, 0, 30, -5])
    traj = generate_joint_trajectory(q_start, q_goal, duration=2.0, n_waypoints=20)

    assert np.allclose(traj["velocities"][0], 0, atol=1e-6)
    assert np.allclose(traj["velocities"][-1], 0, atol=1e-6)
    assert np.allclose(traj["accelerations"][0], 0, atol=1e-6)
    assert np.allclose(traj["accelerations"][-1], 0, atol=1e-6)
    assert np.allclose(traj["positions"][0], q_start, atol=1e-6)
    assert np.allclose(traj["positions"][-1], q_goal, atol=1e-6)


def test_clear_trajectory_passes_collision_check():
    q_start = np.zeros(6)
    q_goal = np.deg2rad([20, -10, 15, 0, 30, -5])
    traj = generate_joint_trajectory(q_start, q_goal)

    valid, msg, idx = validate_trajectory(traj)
    assert valid
    assert idx is None


def test_blocked_trajectory_fails_collision_check():
    q_start = np.zeros(6)
    q_goal = np.deg2rad([30, -20, 45, 0, 60, -10])
    traj = generate_joint_trajectory(q_start, q_goal, n_waypoints=25)

    obstacle = Obstacle(center=[0.15, 0.0, 0.35], radius=0.2)
    valid, msg, idx = validate_trajectory(traj, obstacles=[obstacle])
    assert not valid
    assert idx is not None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
