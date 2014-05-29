cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_COLOR_MAKEFILE ON)
#SET(CMAKE_C_COMPILER "gcc-4.7")
#SET(CMAKE_CXX_COMPILER "g++-4.7")
set(CMAKE_CXX_FLAGS "-std=c++0x")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
                    src/d_model.cpp
                    src/d_model_utils.cpp
                    src/d_model_learner.cpp
                    src/d_model_planner.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(test_d_model src/test/main.cpp)
rosbuild_add_executable(test_env src/test/test_env.cpp)
rosbuild_add_executable(experiments src/test/experiments.cpp)
rosbuild_add_gtest(test/learner_utest test/learner_utest.cpp)
target_link_libraries(test_d_model ${PROJECT_NAME})
target_link_libraries(test_env ${PROJECT_NAME})
target_link_libraries(experiments ${PROJECT_NAME})
target_link_libraries(test/learner_utest ${PROJECT_NAME})
