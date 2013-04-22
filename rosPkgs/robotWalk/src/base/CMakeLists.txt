# generate Trajectories based on straight walk ZMP
cmake_minimum_required (VERSION 2.8)
project ( generateTraj )

message (STATUS "Creating Makefile for zmp generator")

# Eigen
include_directories( /usr/include/eigen3 . )

# Search all header and source files
set ( zmp_src generateTraj.cpp utilities.cpp )
set ( zmp_hdr utilities.h )

add_executable ( generateTraj ${zmp_src} ${zmp_hdr} )