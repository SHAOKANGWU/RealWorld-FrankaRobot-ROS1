cmake_minimum_required(VERSION 3.0.2)
project(panda_spacemouse_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  franka_gripper
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
    GripperCommand.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    franka_gripper
    geometry_msgs
    message_runtime
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/spacemouse_mapper.py

  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
