"""
fk_solver.py
------------
Forward Kinematics for a 6-DOF robotic arm using
Denavit-Hartenberg (DH) parameters.

Forward Kinematics answers: "Given these 6 joint angles,
where is the end-effector (hand)?"
It is DETERMINISTIC — one set of joint angles always gives
exactly one end-effector pose. That's why we solve it first:
it's the ground truth we use to check the IK solver later.

Run standalone to test:
    python3 fk_solver.py
"""

import numpy as np


class DHLink:
    """One row of a Denavit-Hartenberg table."""

    def __init__(self, a, alpha, d, theta_offset=0.0):
        self.a = a                      # link length
        self.alpha = alpha              # link twist (rad)
        self.d = d                      # link offset
        self.theta_offset = theta_offset  # fixed offset added to joint variable


# Example 6-DOF arm (rough dimensions, in meters/radians).
# Replace these with your real robot's measured DH parameters.
DEFAULT_DH_TABLE = [
    DHLink(a=0.0,   alpha=np.pi / 2, d=0.333),
    DHLink(a=0.0,   alpha=-np.pi / 2, d=0.0),
    DHLink(a=0.0,   alpha=np.pi / 2, d=0.316),
    DHLink(a=0.0825, alpha=np.pi / 2, d=0.0),
    DHLink(a=-0.0825, alpha=-np.pi / 2, d=0.384),
    DHLink(a=0.0,   alpha=np.pi / 2, d=0.0),
]


def dh_transform(a, alpha, d, theta):
    """Standard DH homogeneous transformation matrix for one joint."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])


def forward_kinematics(joint_angles, dh_table=DEFAULT_DH_TABLE):
    """
    Compute the end-effector pose from joint angles.

    Args:
        joint_angles: list/array of 6 joint angles (radians)
        dh_table: list of DHLink objects

    Returns:
        T: 4x4 homogeneous transform (base_link -> end_effector)
        joint_positions: list of xyz positions of each joint origin
                          (useful for collision checking / visualization)
    """
    assert len(joint_angles) == len(dh_table), "angle/DH table length mismatch"

    T = np.eye(4)
    joint_positions = [T[:3, 3].copy()]

    for link, theta in zip(dh_table, joint_angles):
        Ti = dh_transform(link.a, link.alpha, link.d, theta + link.theta_offset)
        T = T @ Ti
        joint_positions.append(T[:3, 3].copy())

    return T, joint_positions


def pose_to_xyz(T):
    """Extract just the xyz translation from a 4x4 transform."""
    return T[:3, 3]


if __name__ == "__main__":
    # Sanity check: all zeros -> arm fully extended along its DH chain
    test_angles = [0.0, np.deg2rad(45), np.deg2rad(90), 0.0, np.deg2rad(-45), 0.0]
    T, joints = forward_kinematics(test_angles)

    print("Joint angles (deg):", [round(np.rad2deg(a), 1) for a in test_angles])
    print("End-effector position (m):", np.round(pose_to_xyz(T), 4))
    print("End-effector rotation matrix:\n", np.round(T[:3, :3], 3))
