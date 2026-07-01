"""
motion_node.py
---------------
The ROS 2 "brain" node. Wires together everything else in this
package into the architecture shown in the slide "Synthesis: The
Production-Grade Node":

    Action Client Request
            |
      IK Solver (math)  <-->  TF2 Transform Tree (space)
            |
    Collision Check (FCL-style safety gate)
            |
    FollowJointTrajectory action -> gazebo_ros_control / real hardware

Usage (after building the workspace):
    ros2 run robotic_arm motion_node
    ros2 topic pub /target_pose geometry_msgs/msg/Point "{x: 0.3, y: 0.1, z: 0.4}"

This node SUBSCRIBES to a target xyz point, solves IK, validates
the resulting trajectory for collisions, and publishes it as a
FollowJointTrajectory goal.
"""

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    ROS_AVAILABLE = True
except ImportError:
    # Allows this file to be imported/tested outside a ROS 2 environment.
    ROS_AVAILABLE = False

from .ik_solver import inverse_kinematics_multi_seed
from .trajectory_planner import generate_joint_trajectory
from .collision_checker import validate_trajectory, Obstacle
from .fk_solver import DEFAULT_DH_TABLE

JOINT_NAMES = [f"joint_{i+1}" for i in range(6)]
JOINT_LIMITS = [(-np.pi, np.pi)] * 6


if ROS_AVAILABLE:

    class MotionNode(Node):
        def __init__(self):
            super().__init__("robotic_arm_motion_node")

            self.current_joint_state = np.zeros(6)

            self.joint_state_sub = self.create_subscription(
                JointState, "/joint_states", self._on_joint_state, 10
            )
            self.target_sub = self.create_subscription(
                Point, "/target_pose", self._on_target_pose, 10
            )

            self._action_client = ActionClient(
                self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
            )

            # Example static obstacle in the workspace. In production this
            # would come from a perception node / planning scene, not be
            # hardcoded.
            self.obstacles = [Obstacle(center=[0.3, 0.0, 0.3], radius=0.08)]

            self.get_logger().info("Robotic-Arm motion_node ready. "
                                    "Publish a geometry_msgs/Point to /target_pose to move.")

        def _on_joint_state(self, msg: JointState):
            if len(msg.position) >= 6:
                self.current_joint_state = np.array(msg.position[:6])

        def _on_target_pose(self, msg: Point):
            target = [msg.x, msg.y, msg.z]
            self.get_logger().info(f"Received target: {target}")
            self._plan_and_execute(target)

        def _plan_and_execute(self, target_xyz):
            # --- 1. INVERSE KINEMATICS ---
            success, q_goal, err = inverse_kinematics_multi_seed(
                target_xyz, joint_limits=JOINT_LIMITS
            )
            if not success:
                self.get_logger().error(
                    f"IK failed to converge (residual error={err:.4f}m). Aborting."
                )
                return
            self.get_logger().info(f"IK solved, residual error={err:.6f}m")

            # --- 2. TRAJECTORY GENERATION ---
            trajectory = generate_joint_trajectory(
                self.current_joint_state, q_goal, duration=3.0, n_waypoints=30
            )

            # --- 3. COLLISION VALIDATION ---
            valid, msg, idx = validate_trajectory(
                trajectory, joint_limits=JOINT_LIMITS, obstacles=self.obstacles
            )
            if not valid:
                self.get_logger().error(f"TRAJECTORY ABORT: {msg}")
                return
            self.get_logger().info(msg)

            # --- 4. SEND TO CONTROLLER (real hardware / Gazebo) ---
            self._send_trajectory_goal(trajectory)

        def _send_trajectory_goal(self, trajectory):
            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    "No controller action server found — is Gazebo/hardware running? "
                    "Trajectory computed successfully but not sent."
                )
                return

            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = JOINT_NAMES

            for i, t in enumerate(trajectory["times"]):
                point = JointTrajectoryPoint()
                point.positions = trajectory["positions"][i].tolist()
                point.velocities = trajectory["velocities"][i].tolist()
                point.accelerations = trajectory["accelerations"][i].tolist()
                sec = int(t)
                nanosec = int((t - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                goal_msg.trajectory.points.append(point)

            self.get_logger().info("Sending FollowJointTrajectory goal...")
            self._action_client.send_goal_async(goal_msg)


def main(args=None):
    if not ROS_AVAILABLE:
        print("rclpy not found. Install ROS 2 and source your workspace, "
              "then run: ros2 run robotic_arm motion_node")
        return

    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
