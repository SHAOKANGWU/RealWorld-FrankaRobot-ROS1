# RGB-D Object Localization

This ROS package estimates the 3D position of a manually selected image point.
The user clicks a visible object surface in the RGB image, the corresponding
aligned depth value is deprojected into the camera optical frame, and the
calibrated eye-to-hand transform converts the point into the Panda base frame.

## Run the localizer

Start the RealSense driver with aligned depth enabled. The project startup
launch does this by default. Then run:

```bash
source /home/robot-learn/ros_franka_robotlearn/devel/setup.bash
roslaunch rgbd_object_localization manual_rgbd_base_localizer.launch
```

Alternatively, start the camera, real Panda, gripper, robot-state TF,
calibrated camera transform, and localizer together:

```bash
roslaunch rgbd_object_localization object_localization_bringup.launch \
  robot_ip:=172.16.0.20
```

This complete bringup connects to the real robot. Do not run it alongside
another launch file that already starts the RealSense driver, Franka control,
or the same calibrated camera transform. It intentionally does not start
MoveIt or an arm trajectory controller because they are not required for object
localization. A standalone libfranka program such as `read-once-state` also
cannot connect while `franka_control` owns the robot connection; use ROS topics
or TF to inspect robot state instead.

In the image window:

- Left-click to select an object surface point.
- Press `c` to clear the selection.
- Press `q` to stop the node.

## ROS interfaces

Default input topics:

| Topic | Purpose |
| --- | --- |
| `/camera/color/camera_info` | Color-camera intrinsics and optical frame |
| `/camera/color/image_raw` | RGB image used for selection |
| `/camera/aligned_depth_to_color/image_raw` | Depth registered to the RGB image |

Output topics:

| Topic | Message | Meaning |
| --- | --- | --- |
| `/object_position/camera` | `geometry_msgs/PointStamped` | Selected surface point in `camera_color_optical_frame` |
| `/object_position/base` | `geometry_msgs/PointStamped` | The same physical point in `panda_link0` |

For example:

```bash
rostopic echo -n 1 /object_position/base
```

All output coordinates are in metres.

## Physical meaning of the estimated position

The estimated XYZ value is the position of the **visible surface point selected
in the camera image**, expressed relative to `panda_link0`. It is a point in the
workspace and does not inherently belong to the flange, hand, TCP, or fingers.

For the Panda gripper used in this project, the center contact point between the
closed inner fingertip pads is approximately coincident with `panda_hand_tcp`.
Under this specific closed-gripper convention, the camera-estimated position
can therefore be treated as the desired `panda_hand_tcp` position, or
equivalently the desired center contact position of the closed fingertips.

This equivalence does not apply when using another part of the robot:

- `panda_link8` is the wrist/flange frame and requires the hand offset.
- The gripper body/palm has a different offset from `panda_hand_tcp`.
- The bottom or outer surfaces of the fingers have their own offsets.
- With an open gripper, each finger position depends on the gripper opening.

The current node publishes a 3D point, not a complete 6-DoF object pose. It does
not estimate the object's center or orientation. Motion planning must separately
specify the controlled robot frame, end-effector orientation, approach offset,
and any grasp/contact offset. Use a collision-safe approach pose rather than
moving directly to a surface point.

## Calibration

The default eye-to-hand calibration is stored in:

```text
config/panda_realsense_eye_on_base_eye_on_base.yaml
```

It attaches the RealSense root frame, `camera_link`, to `panda_link0`. The
RealSense driver then supplies its fixed internal transform chain from
`camera_link` to `camera_color_optical_frame`. The resulting composed
`panda_link0` to `camera_color_optical_frame` transform is the refined
calibration used by the localizer. Publishing the calibration at the camera
root is important: directly assigning another parent to
`camera_color_optical_frame` would conflict with the RealSense TF tree.

Contact-based validation across the camera view showed approximately 5--10 mm
practical localization accuracy when depth and manual point selection were
reliable.

Do not run another publisher for the same static transform while
`manual_rgbd_base_localizer.launch` is publishing the calibration.
