cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# find gazebo 
include(FindPkgConfig)
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
else()
  message( FATAL_ERROR "pkg-config is required; please install it" )
endif()

# depends on DRCVehiclePlugin
include_directories( ${GAZEBO_INCLUDE_DIRS} /usr/local/include/eigen-3.0.5 )
link_directories( ${GAZEBO_LIBRARY_DIRS} ) 

rosbuild_add_executable( publish_joint_commands src/publish_joint_commands.cpp )
rosbuild_add_executable( publish_joint_command_fromTxt src/publish_joint_commands_fromTxt.cpp )
rosbuild_add_executable( subscribe_jointStates_test src/subscribe_jointStates_test.cpp )
rosbuild_add_executable( test_keyframes src/test_keyframes.cpp )




