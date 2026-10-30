# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "observation_receiver: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iobservation_receiver:/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Ifranka_msgs:/opt/ros/noetic/share/franka_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(observation_receiver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg" NAME_WE)
add_custom_target(_observation_receiver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "observation_receiver" "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg" "franka_msgs/Errors:sensor_msgs/JointState:franka_msgs/FrankaState:sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg" NAME_WE)
add_custom_target(_observation_receiver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "observation_receiver" "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg" "franka_msgs/Errors:sensor_msgs/JointState:franka_msgs/FrankaState:sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/observation_receiver
)
_generate_msg_cpp(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/observation_receiver
)

### Generating Services

### Generating Module File
_generate_module_cpp(observation_receiver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/observation_receiver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(observation_receiver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(observation_receiver_generate_messages observation_receiver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_cpp _observation_receiver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_cpp _observation_receiver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(observation_receiver_gencpp)
add_dependencies(observation_receiver_gencpp observation_receiver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS observation_receiver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/observation_receiver
)
_generate_msg_eus(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/observation_receiver
)

### Generating Services

### Generating Module File
_generate_module_eus(observation_receiver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/observation_receiver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(observation_receiver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(observation_receiver_generate_messages observation_receiver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_eus _observation_receiver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_eus _observation_receiver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(observation_receiver_geneus)
add_dependencies(observation_receiver_geneus observation_receiver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS observation_receiver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/observation_receiver
)
_generate_msg_lisp(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/observation_receiver
)

### Generating Services

### Generating Module File
_generate_module_lisp(observation_receiver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/observation_receiver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(observation_receiver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(observation_receiver_generate_messages observation_receiver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_lisp _observation_receiver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_lisp _observation_receiver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(observation_receiver_genlisp)
add_dependencies(observation_receiver_genlisp observation_receiver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS observation_receiver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/observation_receiver
)
_generate_msg_nodejs(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/observation_receiver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(observation_receiver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/observation_receiver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(observation_receiver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(observation_receiver_generate_messages observation_receiver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_nodejs _observation_receiver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_nodejs _observation_receiver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(observation_receiver_gennodejs)
add_dependencies(observation_receiver_gennodejs observation_receiver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS observation_receiver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/observation_receiver
)
_generate_msg_py(observation_receiver
  "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/franka_msgs/cmake/../msg/Errors.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/franka_msgs/cmake/../msg/FrankaState.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/observation_receiver
)

### Generating Services

### Generating Module File
_generate_module_py(observation_receiver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/observation_receiver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(observation_receiver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(observation_receiver_generate_messages observation_receiver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/SynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_py _observation_receiver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot-learn/ros_franka_robotlearn/src/observation_receiver/msg/MultiCameraSynchronizedObservation.msg" NAME_WE)
add_dependencies(observation_receiver_generate_messages_py _observation_receiver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(observation_receiver_genpy)
add_dependencies(observation_receiver_genpy observation_receiver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS observation_receiver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/observation_receiver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/observation_receiver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(observation_receiver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(observation_receiver_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET franka_msgs_generate_messages_cpp)
  add_dependencies(observation_receiver_generate_messages_cpp franka_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/observation_receiver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/observation_receiver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(observation_receiver_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(observation_receiver_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET franka_msgs_generate_messages_eus)
  add_dependencies(observation_receiver_generate_messages_eus franka_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/observation_receiver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/observation_receiver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(observation_receiver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(observation_receiver_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET franka_msgs_generate_messages_lisp)
  add_dependencies(observation_receiver_generate_messages_lisp franka_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/observation_receiver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/observation_receiver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(observation_receiver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(observation_receiver_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET franka_msgs_generate_messages_nodejs)
  add_dependencies(observation_receiver_generate_messages_nodejs franka_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/observation_receiver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/observation_receiver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/observation_receiver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(observation_receiver_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(observation_receiver_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET franka_msgs_generate_messages_py)
  add_dependencies(observation_receiver_generate_messages_py franka_msgs_generate_messages_py)
endif()
