# Panda demo recorder

The recorder node saves Franka state, gripper state, and camera images to an
episode directory. For kinesthetic collection, start the complete stack with:

```bash
roslaunch panda_demo_recorder collect_demo_kinesthetic_teaching.launch robot_ip:=ROBOT_IP output_root:=/absolute/path/to/demos
```

For SpaceMouse teleoperation, use:

```bash
roslaunch panda_demo_recorder collect_demo_teleoperation.launch robot_ip:=ROBOT_IP output_root:=/absolute/path/to/teleop_demos
```

This launch reuses `panda_spacemouse_teleop/teleop.launch` and the recorder-only
`demo_recorder.launch`. Both collection modes use the same recorder settings
and save robot state, measured gripper state, and camera images. SpaceMouse
commands are not saved. Start and stop each episode with the recorder's
`/demo_recorder/start` and `/demo_recorder/stop` Trigger services.

`robot_ip` and `output_root` are per-run launch arguments. Stable topic names,
JPEG quality, controller settings, and SpaceMouse mapping live in their YAML
files. Use `config_file`, `controller_config_file`, or
`spacemouse_config_file` to select another configuration. `load_gripper` and
`start_controller` remain launch switches.
