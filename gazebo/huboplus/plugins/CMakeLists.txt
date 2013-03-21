cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} . ) # /usr/include/gazebo-1.4
link_directories(${GAZEBO_LIBRARY_DIRS})

set( control_srcs controls/pid_controller.cpp controls/controlBundle.cpp )
set( gaiter_srcs gaiter/gaiter.cpp )

add_library(foreleg SHARED foreLeg/foreleg_jointControl.cpp ${control_srcs} ${gaiter_srcs} )
target_link_libraries( foreleg ${GAZEBO_LIBRARIES})

add_library(cog_visual SHARED foreLeg/cog_visual.cpp )
target_link_libraries( cog_visual ${GAZEBO_LIBRARIES})

add_library(swing_plugin SHARED swing/swing_plugin.cpp )
target_link_libraries( swing_plugin ${GAZEBO_LIBRARIES})

add_library(hiWorld_plugin SHARED snippets/hiWorld_plugin.cpp )
target_link_libraries( hiWorld_plugin ${GAZEBO_LIBRARIES})
