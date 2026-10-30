# Reference header from the first configured camera.
std_msgs/Header header
# Parallel arrays in configuration order: camera_names[i] identifies images[i].
string[] camera_names
sensor_msgs/Image[] images
franka_msgs/FrankaState state
sensor_msgs/JointState gripper_state
