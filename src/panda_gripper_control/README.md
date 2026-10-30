# Panda keyboard gripper control

This ROS 1 package controls the Franka gripper through its standard action
servers. It is intended to run alongside the kinesthetic teaching and demo
recorder launch.

## Build

```bash
cd ~/ros_franka_robotlearn
catkin_make --pkg panda_gripper_control
source devel/setup.bash
```

## Run

Start the robot, camera, kinesthetic controller, and recorder in one terminal:

```bash
roslaunch panda_demo_recorder demo_recorder.launch robot_ip:=172.16.0.20
```

Start the keyboard client in a second interactive terminal:

```bash
cd ~/ros_franka_robotlearn
source devel/setup.bash
roslaunch panda_gripper_control keyboard_gripper.launch
```

The default keys are:

- `g`: grasp
- `r`: release/open
- `s`: stop the current gripper motion
- `q`: stop and quit

Widths, speed, force, grasp tolerances, action names, and keyboard mappings are
configured in `config/keyboard_gripper.yaml`.
