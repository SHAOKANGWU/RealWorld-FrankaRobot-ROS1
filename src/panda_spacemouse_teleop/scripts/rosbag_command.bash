# argument: demo number
DEMO_NUM=$1

rosbag record \
  -O demo_${DEMO_NUM}.bag \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /franka_state_controller/franka_states \
  /franka_gripper/joint_states \
  /teleop/cartesian_velocity_cmd \
  /teleop/gripper_command
