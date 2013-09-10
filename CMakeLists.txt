#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola
#  Copyright 2010 CNRS
#


cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/base.cmake)
include(cmake/GNUInstallDirs.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/src)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#FINDPYTHON()
#INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
#LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

add_required_dependency("jrl-mal")
add_required_dependency("dynamic-graph")
add_required_dependency("dynamic-graph-python")
add_required_dependency("sot-core")
add_required_dependency("sot-dynamic")


#file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/${NAME}")
rosbuild_add_library(ros_sot_robot_model src/ros_sot_robot_model.cpp include/ros_sot_robot_model/ros_sot_robot_model.hh)

target_link_libraries(ros_sot_robot_model dynamic-graph)
target_link_libraries(ros_sot_robot_model dynamic-graph-python)
target_link_libraries(ros_sot_robot_model ros_bridge)
#set_target_properties(ros_sot_robot_model PROPERTIES BUILD_WITH_INSTALL_RPATH True)
#install(TARGETS ros_sot_robot_model DESTINATION build)

PKG_CONFIG_USE_DEPENDENCY(ros_sot_robot_model "jrl-mal")
PKG_CONFIG_USE_DEPENDENCY(ros_sot_robot_model "dynamic_graph")
PKG_CONFIG_USE_DEPENDENCY(ros_sot_robot_model "sot-core")
  
ADD_SUBDIRECTORY(src)  
  
#PYTHON_INSTALL_BUILD(MODULE FILE DEST)







