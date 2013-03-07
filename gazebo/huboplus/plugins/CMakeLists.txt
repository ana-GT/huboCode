cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} ) # /usr/include/gazebo-1.4
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(foreleg SHARED foreLeg/foreleg_jointControl.cpp pid_controller.cpp controlBundle.cpp )
target_link_libraries( foreleg ${GAZEBO_LIBRARIES})

add_library(cog_visual SHARED foreLeg/cog_visual.cpp )
target_link_libraries( cog_visual ${GAZEBO_LIBRARIES})

