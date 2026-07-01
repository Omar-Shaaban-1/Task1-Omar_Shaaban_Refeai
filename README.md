# Robotic-Arm

A simulated 6-DOF robotic arm that moves from Point A to Point B
using Inverse Kinematics, generates a smooth collision-free
trajectory, and executes it in ROS 2 + Gazebo.

---

## What's in this repo

```
Robotic-Arm/
├── package.xml                  <- ROS 2 package manifest
├── setup.py / setup.cfg         <- Python package build config
├── resource/robotic_arm         <- ROS 2 package marker file
├── robotic_arm/                 <- the actual logic
│   ├── fk_solver.py             <- Forward Kinematics (DH parameters)
│   ├── ik_solver.py             <- Inverse Kinematics (damped least squares)
│   ├── trajectory_planner.py    <- Quintic spline trajectory generation
│   ├── collision_checker.py     <- Joint limit / self-collision / obstacle checks
│   └── motion_node.py           <- ROS 2 node wiring it all together
├── urdf/robotic_arm.urdf.xacro  <- Robot description (links, joints, transmissions)
├── launch/
│   ├── display.launch.py        <- RViz visualization (no physics)
│   └── gazebo.launch.py         <- Full physics simulation
├── config/controllers.yaml      <- PID gains for each joint
└── test/test_kinematics.py      <- Unit tests (no ROS required)
README.md                    <- you are here
```

## How the pieces fit together (the pipeline)

```
target xyz  ─▶  IK Solver  ─▶  Trajectory Planner  ─▶  Collision Checker  ─▶  Gazebo/Hardware
             (ik_solver.py)   (trajectory_planner.py)  (collision_checker.py)   (motion_node.py)
```

1. **`fk_solver.py`** — the ground truth. Given 6 joint angles, computes
   exactly where the end-effector ends up. Everything else is validated
   against this.
2. **`ik_solver.py`** — the hard part. Given a target xyz, iteratively
   searches for joint angles that reach it (damped least-squares Jacobian
   method — the same family of algorithm as TRAC-IK, just hand-rolled
   so you can see every step).
3. **`trajectory_planner.py`** — turns "start angles → goal angles" into
   a smooth quintic-polynomial path with zero velocity/acceleration at
   the endpoints (no jerky starts/stops).
4. **`collision_checker.py`** — walks every waypoint and rejects the
   trajectory if any joint exceeds its limits, gets too close to itself,
   or intersects an obstacle.
5. **`motion_node.py`** — the ROS 2 node that chains 1→4 together and
   sends the final trajectory to the real/simulated hardware via the
   standard `FollowJointTrajectory` action.

---

## What to do — step by step

### Phase 1: Validate the math (no ROS needed, do this first)

You can test everything except actual robot motion right now, with
just Python:

```bash
cd Robotic-Arm
pip install numpy pytest --break-system-packages

# Run each module standalone to see it work:
python3 robotic_arm/fk_solver.py
python3 robotic_arm/ik_solver.py
python3 robotic_arm/trajectory_planner.py
python3 robotic_arm/collision_checker.py

# Run the full test suite:
python3 -m pytest test/test_kinematics.py -v
```

All 5 tests should pass. This proves your FK/IK round-trip correctly,
your trajectories start/end at rest, and your collision checker
catches obstacles.

**→ Do this before touching ROS.** If the math is wrong, no amount of
ROS debugging will fix it.

### Phase 2: Set up ROS 2 (if you don't have it yet)

Install ROS 2 (Humble or Jazzy) on Ubuntu 22.04/24.04:
- Follow the official docs at https://docs.ros.org/ for your distro.
- Then install the extra packages this project needs:

```bash
sudo apt install ros-<distro>-gazebo-ros-pkgs \
                  ros-<distro>-gazebo-ros2-control \
                  ros-<distro>-joint-state-publisher-gui \
                  ros-<distro>-robot-state-publisher \
                  ros-<distro>-xacro \
                  ros-<distro>-joint-trajectory-controller \
                  ros-<distro>-rviz2
```

### Phase 3: Build the workspace

```bash
mkdir -p ~/robotic_arm_ws/src
cp -r Robotic-Arm ~/robotic_arm_ws/src/robotic_arm
cd ~/robotic_arm_ws
colcon build
source install/setup.bash
```

### Phase 4: Visualize in RViz (kinematic intent, no physics)

```bash
ros2 launch robotic_arm display.launch.py
```

Use the joint_state_publisher_gui sliders to manually move each
joint and confirm the arm looks right — this is the "does my URDF
make sense" sanity check, before adding gravity/physics.

### Phase 5: Simulate in Gazebo (physical reality)

```bash
ros2 launch robotic_arm gazebo.launch.py
```

Watch the arm settle under gravity. If it sags or drifts, your PID
gains in `config/controllers.yaml` are too low — increase P. If it
shakes/oscillates, they're too high — decrease P and/or add D.

### Phase 6: Command the arm to move

In a separate terminal, once Gazebo is running:

```bash
ros2 run robotic_arm motion_node
```

Then send it a target:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/Point "{x: 0.3, y: 0.1, z: 0.4}"
```

Watch the terminal log the pipeline in action:
`IK solved → trajectory generated → collision check passed → goal sent to Gazebo`

### Phase 7: Break it on purpose (this is how you learn)

- Send a target way outside the arm's reach → watch IK fail gracefully.
- Edit the obstacle in `motion_node.py` to sit directly in the arm's
  path → watch the collision checker abort the trajectory.
- Push the `p` gain in `controllers.yaml` way up → watch it oscillate
  in Gazebo. Then tune it back down properly.
- Try a different interpolation (e.g. cubic instead of quintic) in
  `trajectory_planner.py` and compare velocity/acceleration profiles.

### Phase 8: Document and ship

- Record a short screen capture of Gazebo executing a pick-style motion.
- Write up your PID tuning process and final gains.
- Push this repo to GitHub as `Robotic-Arm` — this is your portfolio
  piece for Project 1.

---

## Notes on the IK solver

This project ships a **hand-rolled damped least-squares IK solver**
(`ik_solver.py`) instead of requiring `TRAC-IK` to be installed,
so you can run and test the math anywhere with just `numpy`. It uses
multi-seed random restarts to avoid the "local minima trap" described
in project materials (where joint-limit clamping breaks the gradient
descent search direction).

If you want production-grade performance later, swap it for
`trac_ik_python`:
```bash
sudo apt install ros-<distro>-trac-ik-kinematics-plugin
pip install trac_ik_python --break-system-packages
```
and replace the call in `motion_node.py` — the rest of the pipeline
(trajectory planner, collision checker) doesn't need to change.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| IK never converges | Target is outside the arm's reachable workspace, or joint limits are too tight |
| Arm sags in Gazebo | P gain too low in `controllers.yaml` |
| Arm shakes/vibrates | P gain too high — reduce it, add D |
| "Collision Detected" every time | `min_link_separation` in `collision_checker.py` may be too generous for your URDF's actual link sizes — measure and adjust |
| `xacro` command not found | `sudo apt install ros-<distro>-xacro` |

