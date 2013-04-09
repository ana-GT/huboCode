## CMake for preview control for ZMP
cmake_minimum_required( VERSION 2.8 )
project( previewZMP )


message( STATUS "Creating makefile for previewZMP" )

#Eigen
include_directories( /usr/include/eigen3 . )

#Search all header and source files
set( previewZMP_hdrs previewZMP.h )
set( previewZMP_srcs previewZMP.cpp test.cpp )

add_executable( previewTest ${previewZMP_hdrs} ${previewZMP_srcs} )