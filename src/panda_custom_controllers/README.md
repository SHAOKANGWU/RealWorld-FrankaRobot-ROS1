# Panda custom controllers

This package provides these pluginlib ROS-control controllers:

- `CartesianVelocityTeleopController`: consumes `geometry_msgs/TwistStamped`
  commands and drives Franka's Cartesian velocity interface with magnitude,
  acceleration, frame, finite-value, and timeout checks.
- `KinestheticTeachingController`: applies gravity and Coriolis compensation for
  hand-guided demonstrations.
- `JointPositionTorqueController`: tracks joint-position commands with a
  joint-space torque controller.
- `CartesianPoseTorqueController`: tracks end-effector pose commands with a
  Cartesian impedance controller.
- `NullSpaceTorqueController`: tracks a Cartesian pose while applying a
  secondary joint-space objective in the Jacobian null space.

The main operator workflow is documented in
`panda_demo_recorder/README.md`. The launch file
`cartesian_velocity_teleop_controller.launch` starts the teleoperation controller
by default. Pass `start_controller:=false` to skip the controller spawner; the
controller parameters are loaded, but the controller itself is neither loaded
nor started. Load it separately through `controller_manager` if a stopped
controller is required.

Supply `robot_ip:=ROBOT_IP` when starting a controller launch. The controllers
and joint lists in this package are configured for a Panda arm. Stable
controller settings live in `config/*.yaml`; pass
`config_file:=/absolute/path/to/custom.yaml` to select another file. The
kinesthetic controller YAML also sets the Franka state publish rate to 50 Hz.
Use launch arguments for session choices such as `load_gripper` and
`start_controller`.
