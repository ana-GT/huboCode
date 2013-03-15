# ******************
# CMake for swing
# ******************

cmake_minimum_required (VERSION 2.8)
project (swing)

message (STATUS "Creating Makefile for swing")

# Include directories
include_directories( . )

# Search all header and source files
set ( swing_src swing.cpp )
set ( swing_hdr swing.h )

add_executable (swingTest test.cpp ${swing_src} ${swing_hdr} )

#target_link_libraries (swing ${SOMETHING} )




