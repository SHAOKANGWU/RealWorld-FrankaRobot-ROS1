cmake_minimum_required(VERSION 3.0.2)
project(panda_custom_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  franka_hw
  actionlib
  franka_gripper
  message_generation
  std_msgs
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  CartesianPoseCommand.msg
  JointPositionCommand.msg
  NullSpaceCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES panda_custom_controllers
  CATKIN_DEPENDS
    actionlib
    controller_interface
    franka_gripper
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    franka_hw
    std_msgs
    message_runtime
    DEPENDS EIGEN3
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_library(panda_custom_controllers
  src/cartesian_velocity_teleop_controller.cpp
  src/kinesthetic_teaching_controller.cpp
  src/cartesian_pose_torque_controller.cpp
  src/joint_position_torque_controller.cpp
  src/null_space_torque_controller.cpp
)

target_link_libraries(panda_custom_controllers
  ${catkin_LIBRARIES}
)

add_dependencies(panda_custom_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS panda_custom_controllers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES panda_custom_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
