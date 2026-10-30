cmake_minimum_required(VERSION 3.0.2)
project(panda_gripper_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  franka_gripper
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    franka_gripper
    rospy
)

catkin_install_python(
  PROGRAMS
    scripts/keyboard_gripper_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
