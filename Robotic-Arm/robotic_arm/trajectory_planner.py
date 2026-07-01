"""
trajectory_planner.py
----------------------
Turns a start/end joint configuration into a smooth,
time-parameterized trajectory (position, velocity, acceleration
at each waypoint) — this is the "Output Stage" from the slides.

We use QUINTIC (5th order) polynomial interpolation per joint,
because it guarantees zero velocity AND zero acceleration at
the start and end (no jerky snaps when the arm starts/stops).

This mirrors control_msgs/action/FollowJointTrajectory, the
real ROS 2 message type used to command the arm.
"""

import numpy as np


def quintic_coefficients(q0, qf, T):
    """
    Solve for the 6 coefficients of a quintic polynomial
    q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    subject to: q(0)=q0, q(T)=qf, v(0)=v(T)=0, a(0)=a(T)=0
    """
    c0 = q0
    c1 = 0.0
    c2 = 0.0
    c3 = (10 * (qf - q0)) / T ** 3
    c4 = (-15 * (qf - q0)) / T ** 4
    c5 = (6 * (qf - q0)) / T ** 5
    return np.array([c0, c1, c2, c3, c4, c5])


def evaluate_quintic(coeffs, t):
    """Return (position, velocity, acceleration) at time t."""
    c0, c1, c2, c3, c4, c5 = coeffs
    pos = c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5
    vel = c1 + 2 * c2 * t + 3 * c3 * t ** 2 + 4 * c4 * t ** 3 + 5 * c5 * t ** 4
    acc = 2 * c2 + 6 * c3 * t + 12 * c4 * t ** 2 + 20 * c5 * t ** 3
    return pos, vel, acc


def generate_joint_trajectory(q_start, q_goal, duration=3.0, n_waypoints=50):
    """
    Generate a smooth multi-joint trajectory.

    Args:
        q_start: array of starting joint angles (radians)
        q_goal: array of goal joint angles (radians)
        duration: total time to execute the motion (seconds)
        n_waypoints: how many timesteps to sample

    Returns:
        dict with keys: times, positions, velocities, accelerations
        positions/velocities/accelerations are shape (n_waypoints, n_joints)
    """
    q_start = np.array(q_start, dtype=float)
    q_goal = np.array(q_goal, dtype=float)
    n_joints = len(q_start)

    coeffs_per_joint = [
        quintic_coefficients(q_start[j], q_goal[j], duration) for j in range(n_joints)
    ]

    times = np.linspace(0, duration, n_waypoints)
    positions = np.zeros((n_waypoints, n_joints))
    velocities = np.zeros((n_waypoints, n_joints))
    accelerations = np.zeros((n_waypoints, n_joints))

    for i, t in enumerate(times):
        for j in range(n_joints):
            pos, vel, acc = evaluate_quintic(coeffs_per_joint[j], t)
            positions[i, j] = pos
            velocities[i, j] = vel
            accelerations[i, j] = acc

    return {
        "times": times,
        "positions": positions,
        "velocities": velocities,
        "accelerations": accelerations,
    }


def to_ros_trajectory_msg(trajectory, joint_names):
    """
    Convert the trajectory dict into a
    trajectory_msgs/msg/JointTrajectory - compatible structure.

    Kept dependency-free here (no rclpy import) so this file can be
    unit-tested outside ROS. motion_node.py wraps this into the real
    ROS 2 message type when actually publishing.
    """
    points = []
    for i, t in enumerate(trajectory["times"]):
        points.append({
            "positions": trajectory["positions"][i].tolist(),
            "velocities": trajectory["velocities"][i].tolist(),
            "accelerations": trajectory["accelerations"][i].tolist(),
            "time_from_start": float(t),
        })
    return {"joint_names": joint_names, "points": points}


if __name__ == "__main__":
    q_start = np.zeros(6)
    q_goal = np.deg2rad([30, -20, 45, 0, 60, -10])

    traj = generate_joint_trajectory(q_start, q_goal, duration=2.5, n_waypoints=10)

    print(f"{'t':>6} | " + " | ".join(f"j{j}_pos(deg)" for j in range(6)))
    for i, t in enumerate(traj["times"]):
        row = " | ".join(f"{np.rad2deg(p):9.2f}" for p in traj["positions"][i])
        print(f"{t:6.2f} | {row}")

    print("\nMax velocity per joint (rad/s):",
          np.round(np.max(np.abs(traj["velocities"]), axis=0), 3))
    print("Max acceleration per joint (rad/s^2):",
          np.round(np.max(np.abs(traj["accelerations"]), axis=0), 3))
