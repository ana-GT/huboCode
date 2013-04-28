# CMakeLists.txt for hubomz
cmake_minimum_required(VERSION 2.6)

project(atlasZmp)

# Include PkgConfig
include(FindPkgConfig)

#Find Eigen
pkg_search_module(EIGEN3 REQUIRED eigen3>=3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Build library atlaszmp
add_subdirectory( tinyWalker )

# Build executable
link_directories( "${CMAKE_SOURCE_DIR}/lib" )
include_directories( . tinyWalker )

add_executable( walkerTrial trial.cpp )
target_link_libraries( walkerTrial atlaszmp )
