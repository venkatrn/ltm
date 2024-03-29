cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-std=c++0x")

rosbuild_init()
#find_package(PCL 1.7 REQUIRED)
#include_directories(BEFORE ${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#if (NOT PCL_FOUND)
#  MESSAGE(FATAL_ERROR "PCL not found.\n")
#endif (NOT PCL_FOUND)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/point_tracker.cpp
  src/perception_interface.cpp
  src/perception_utils.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(point_tracker_node src/point_tracker_node.cpp)
rosbuild_add_executable(ltm_perception_node src/ltm_perception_node.cpp)
target_link_libraries(point_tracker_node ${PROJECT_NAME})
target_link_libraries(point_tracker_node libvtkCommon.so libvtkFiltering.so )
target_link_libraries(ltm_perception_node ${PROJECT_NAME})
target_link_libraries(ltm_perception_node libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
#target_link_libraries(point_tracker_node ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so )
