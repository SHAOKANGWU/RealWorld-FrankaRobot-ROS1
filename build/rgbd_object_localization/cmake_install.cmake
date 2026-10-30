# Install script for directory: /home/robot-learn/ros_franka_robotlearn/src/rgbd_object_localization

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/robot-learn/ros_franka_robotlearn/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robot-learn/ros_franka_robotlearn/build/rgbd_object_localization/catkin_generated/installspace/rgbd_object_localization.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rgbd_object_localization/cmake" TYPE FILE FILES
    "/home/robot-learn/ros_franka_robotlearn/build/rgbd_object_localization/catkin_generated/installspace/rgbd_object_localizationConfig.cmake"
    "/home/robot-learn/ros_franka_robotlearn/build/rgbd_object_localization/catkin_generated/installspace/rgbd_object_localizationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rgbd_object_localization" TYPE FILE FILES "/home/robot-learn/ros_franka_robotlearn/src/rgbd_object_localization/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rgbd_object_localization" TYPE PROGRAM FILES "/home/robot-learn/ros_franka_robotlearn/build/rgbd_object_localization/catkin_generated/installspace/manual_rgbd_localizer.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rgbd_object_localization" TYPE PROGRAM FILES "/home/robot-learn/ros_franka_robotlearn/build/rgbd_object_localization/catkin_generated/installspace/manual_rgbd_base_localizer.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rgbd_object_localization" TYPE PROGRAM FILES "/home/robot-learn/ros_franka_robotlearn/build/rgbd_object_localization/catkin_generated/installspace/identify_aruco_dictionary.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rgbd_object_localization" TYPE PROGRAM FILES "/home/robot-learn/ros_franka_robotlearn/build/rgbd_object_localization/catkin_generated/installspace/aruco_pose_publisher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rgbd_object_localization" TYPE DIRECTORY FILES
    "/home/robot-learn/ros_franka_robotlearn/src/rgbd_object_localization/launch"
    "/home/robot-learn/ros_franka_robotlearn/src/rgbd_object_localization/config"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rgbd_object_localization" TYPE FILE FILES
    "/home/robot-learn/ros_franka_robotlearn/src/rgbd_object_localization/README.md"
    "/home/robot-learn/ros_franka_robotlearn/src/rgbd_object_localization/requirements.txt"
    )
endif()

