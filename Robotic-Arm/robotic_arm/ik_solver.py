"""
ik_solver.py
------------
Inverse Kinematics for the 6-DOF arm.

Inverse Kinematics answers: "What joint angles put the hand
at THIS xyz point?" Unlike FK, this is NON-LINEAR and can have
zero, one, or many valid solutions. We solve it iteratively:

  1. Start from a guess (seed) joint configuration.
  2. Compute FK -> see how far off we are from the target.
  3. Compute the Jacobian (how each joint's tiny movement
     changes the end-effector position).
  4. Use damped least squares (a stabilized Jacobian inverse)
     to step the joint angles closer to the target.
  5. Repeat until the error is tiny or we give up.

This is the same family of method used by Orocos-KDL /
TRAC-IK in real ROS systems. In production you would swap
this for the `trac_ik_python` package, which is faster and
more robust — but this hand-rolled version teaches you exactly
what's happening under the hood, and works with zero extra
ROS dependencies.

Run standalone to test:
    python3 ik_solver.py
"""

import numpy as np
from fk_solver import forward_kinematics, DEFAULT_DH_TABLE, pose_to_xyz


def numerical_jacobian(joint_angles, dh_table=DEFAULT_DH_TABLE, eps=1e-6):
    """
    Approximate the 3x6 position Jacobian by finite differences.
    J[:, i] = d(end_effector_xyz) / d(joint_angle_i)
    """
    n = len(joint_angles)
    J = np.zeros((3, n))
    T0, _ = forward_kinematics(joint_angles, dh_table)
    p0 = pose_to_xyz(T0)

    for i in range(n):
        perturbed = list(joint_angles)
        perturbed[i] += eps
        Ti, _ = forward_kinematics(perturbed, dh_table)
        pi = pose_to_xyz(Ti)
        J[:, i] = (pi - p0) / eps

    return J, p0


def inverse_kinematics(
    target_xyz,
    seed_angles=None,
    dh_table=DEFAULT_DH_TABLE,
    joint_limits=None,
    max_iters=200,
    tol=1e-4,
    damping=0.05,
):
    """
    Solve for joint angles that place the end-effector at target_xyz.

    Args:
        target_xyz: [x, y, z] target position (meters)
        seed_angles: initial guess (radians). Defaults to all zeros.
        joint_limits: list of (min, max) tuples per joint, radians.
                      If a joint hits a limit it is clamped (this is
                      the "Local Minima Trap" from the slides — clamping
                      can break convergence, which is why multiple seeds
                      are tried below).
        max_iters: iterations per attempt
        tol: acceptable position error (meters)
        damping: damped least squares lambda (stability vs. speed)

    Returns:
        (success: bool, joint_angles: np.array, final_error: float)
    """
    n = len(dh_table)
    target_xyz = np.array(target_xyz, dtype=float)

    if seed_angles is None:
        seed_angles = np.zeros(n)
    q = np.array(seed_angles, dtype=float)

    if joint_limits is None:
        joint_limits = [(-np.pi, np.pi)] * n

    for _ in range(max_iters):
        J, p_current = numerical_jacobian(q, dh_table)
        error = target_xyz - p_current
        err_norm = np.linalg.norm(error)

        if err_norm < tol:
            return True, q, err_norm

        # Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 * error
        JJt = J @ J.T
        lam_sq = damping ** 2
        dq = J.T @ np.linalg.solve(JJt + lam_sq * np.eye(3), error)

        q = q + dq

        # Clamp to joint limits (this is where solvers can get "trapped")
        for i, (lo, hi) in enumerate(joint_limits):
            q[i] = np.clip(q[i], lo, hi)

    # Did not converge within max_iters
    _, p_final = numerical_jacobian(q, dh_table)
    final_error = np.linalg.norm(target_xyz - p_final)
    return final_error < tol, q, final_error


def inverse_kinematics_multi_seed(target_xyz, dh_table=DEFAULT_DH_TABLE,
                                   joint_limits=None, n_attempts=8):
    """
    Try several random seed configurations to avoid the local-minima
    trap described in the slides (det(J) -> 0 / joint-limit clamping
    breaking the search direction). Mirrors what TRAC-IK does with
    its concurrent solver restarts.
    """
    best = (False, None, np.inf)

    for attempt in range(n_attempts):
        if attempt == 0:
            seed = np.zeros(len(dh_table))
        else:
            seed = np.random.uniform(-np.pi, np.pi, len(dh_table))

        success, q, err = inverse_kinematics(
            target_xyz, seed_angles=seed, dh_table=dh_table, joint_limits=joint_limits
        )

        if success:
            return True, q, err
        if err < best[2]:
            best = (False, q, err)

    return best


if __name__ == "__main__":
    # Use FK to generate a *known reachable* target, then recover it with IK.
    true_angles = [0.1, 0.7, -0.3, 0.2, 0.5, -0.1]
    T_true, _ = forward_kinematics(true_angles)
    target = pose_to_xyz(T_true)

    print("Target xyz:", np.round(target, 4))

    success, q_solution, err = inverse_kinematics_multi_seed(target)

    print("IK converged:", success)
    print("Solved joint angles (deg):", np.round(np.rad2deg(q_solution), 2))
    print("Final position error (m):", round(err, 6))

    # Verify: plug solution back into FK
    T_check, _ = forward_kinematics(q_solution)
    print("FK(IK(target)) xyz:", np.round(pose_to_xyz(T_check), 4))
