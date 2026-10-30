# Move to a recorded start point

Both move-to-start scripts can read the first nonempty row of the 15-column
`trajectory.txt` produced by `panda_demo_recorder/convert_trajectory_to_txt.py`:

```text
x y z qx qy qz qw joint1 joint2 joint3 joint4 joint5 joint6 joint7 gripper
```

The pose script also accepts a row containing only `x y z qx qy qz qw`.

Each move-to-start launch file includes
`panda_moveit_config/franka_control.launch`, which starts `franka_control`,
the position joint trajectory controller, and `move_group` with the real
controller manager. Supply the physical robot's IP. Launch one reset node at
a time; do not separately start another `franka_control` instance:

```bash
roslaunch panda_reproduction move_to_start_joint.launch robot_ip:=ROBOT_IP trajectory_file:=/absolute/path/to/trajectory.txt dry_run:=true
roslaunch panda_reproduction move_to_start_pose.launch robot_ip:=ROBOT_IP trajectory_file:=/absolute/path/to/trajectory.txt dry_run:=true
```

Set `dry_run:=false` to execute the planned reset. The reset launch does not
start trajectory reproduction. After the pose reset succeeds, stop that launch
before starting the Cartesian replay launch; both launches start
`franka_control`, so they must not run at the same time:

```bash
roslaunch panda_reproduction cartesian_pose_reproduction.launch robot_ip:=ROBOT_IP trajectory_file:=/absolute/path/to/trajectory.txt
```

The replay launch starts the Cartesian torque controller and publisher. The
publisher checks that the controller is running and the robot is at the first
recorded end-effector pose before sending the trajectory. The joint replay
launch similarly starts its joint torque controller:

```bash
roslaunch panda_reproduction joint_position_reproduction.launch robot_ip:=ROBOT_IP trajectory_file:=/absolute/path/to/trajectory.txt
```

The joint and Cartesian replay YAML files keep stable playback settings and
default `trajectory_file` to empty. Supply the recorded file through the
`trajectory_file:=...` launch argument for each run. A custom replay YAML can
instead provide inline `trajectory` points; leave the file argument empty in
that case. Replay refuses to publish when neither source is configured.
Joint replay also refuses to publish unless every measured joint is within the
configured tolerance of the first waypoint. Null-space replay checks the first
waypoint's end-effector pose and all seven joints. Use the joint move-to-start
launch before either replay when the check fails.

The reset launch files likewise load stable planning settings from their
matching YAML files, then set `trajectory_file` and `dry_run` from launch
arguments. For all launches, `config_file:=/absolute/path/to/custom.yaml`
selects another settings file. Joint and Cartesian replay also accept
`controller_config_file:=/absolute/path/to/controller.yaml` for controller
gains. `robot_ip` identifies the arm, and `load_gripper:=true` is the default.
The null-space replay launch accepts `trajectory_file` but only starts its
publisher; start the matching controller and robot separately before using it.

The scripts also keep their direct CLI interface when the real-robot MoveIt
stack is already running:

```bash
rosrun panda_reproduction move_to_start_joint.py /absolute/path/to/trajectory.txt --dry-run
rosrun panda_reproduction move_to_start_joint.py /absolute/path/to/trajectory.txt

rosrun panda_reproduction move_to_start_pose.py /absolute/path/to/trajectory.txt --dry-run
rosrun panda_reproduction move_to_start_pose.py /absolute/path/to/trajectory.txt
```

The joint script uses the first row's seven recorded joint positions. The pose
script uses only its end-effector position and quaternion; it also accepts a
seven-value pose-only row. It ignores the recorded joint positions and gripper
action. Neither script moves the gripper.

The pose script converts the recorded Franka `O_T_EE` target to MoveIt's
`panda_link8` flange target using the live `F_T_EE` tool transform. It checks
that MoveIt's current flange pose agrees with Franka state before planning,
then verifies only the end-effector pose after execution. A pose target may
reach a different joint configuration; use the joint reset script when that
configuration matters. `--dry-run` plans and checks the path without
executing it. MoveIt checks collisions against its
planning scene, so add any relevant obstacles to that scene before planning.
The scripts require a running `position_joint_trajectory_controller` and refuse
to execute through MoveIt's fake controller manager or while another arm
controller is active. Both reset scripts leave the position controller running;
stop their launch before starting a standalone reproduction launch.

## Evaluate joint-reproduction accuracy

Record the desired joint command and measured Franka state on the robot PC,
then compare them offline with the exact trajectory used for replay:

```bash
rosrun panda_reproduction analyze_joint_reproduction.py \
  /absolute/path/to/joint_reproduction_accuracy.bag \
  /absolute/path/to/trajectory.txt
```

The analyzer uses message timestamps and the controller's zero-order command
hold. It excludes repeated final-hold commands from dynamic metrics, estimates
constant tracking delay, and reports timestamp-aligned, delay-compensated, and
settled hold errors. It writes `summary.json`, `joint_metrics.csv`,
`joint_tracking.png`, and `joint_errors.png` beside the bag by default. Pass
`--output-dir` to select another directory.

For Cartesian-pose reproduction, record the Cartesian command and Franka
state, then run:

```bash
rosrun panda_reproduction analyze_cartesian_reproduction.py \
  /absolute/path/to/cartesian_reproduction_accuracy.bag \
  /absolute/path/to/trajectory.txt
```

This analyzer reports translation and quaternion geodesic errors separately,
including their best constant delays and settled final-pose errors. It writes
`summary.json`, `cartesian_tracking.png`, and `cartesian_errors.png` beside the
bag by default.

By default, MoveIt velocity and acceleration scaling are both 0.05; use
`--velocity-scaling` and `--acceleration-scaling` to adjust them. The joint
script checks the recorded joints against the URDF limits and restricts the
planned joint excursion to 1 rad by default. The pose script instead limits
the requested position change to 0.30 m and orientation change to 1.57 rad.
Its final pose tolerances default to 1 cm and 0.05 rad. Both scripts require
the arm to be still before planning. Use `--help` to see all parameters.
