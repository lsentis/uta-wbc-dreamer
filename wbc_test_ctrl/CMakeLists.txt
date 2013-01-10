cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_BUILD_TYPE Debug)
set (ROS_BUILD_TYPE Debug)
add_definitions (-Wall -pipe)
rosbuild_init ()
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


include (CheckIncludeFileCXX)

rosbuild_add_library (wbc_test_ctrl
  src/test_util_base.cpp
  src/comm_udp.c
  )
  
rosbuild_add_executable (servo_base src/servo_base.cpp)
target_link_libraries (servo_base wbc_test_ctrl)

