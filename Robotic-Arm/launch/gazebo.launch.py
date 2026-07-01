"""
gazebo.launch.py
------------------
Spawns the arm into Gazebo with physics enabled (gravity, mass,
contact dynamics) and starts the ros2_control controllers. This
is the "Gazebo (Physical Reality)" half of the Split-Reality Mirror
— it shows what the robot will ACTUALLY do, PID tuning and all.

Run:
    ros2 launch robotic_arm gazebo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robotic_arm")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    urdf_path = os.path.join(pkg_share, "urdf", "robotic_arm.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robotic_arm"],
        output="screen",
    )

    # Load and start the joint trajectory controller once the robot exists in sim
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )

    load_arm_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "arm_controller"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
    ])
