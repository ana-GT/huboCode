cmake_minimum_required (VERSION 2.6)
project (basicModel)

message (STATUS "Creating Makefile for basicModel")

# Eigen
include_directories( /usr/include/eigen3 . )

# Search all header and source files
set ( basicModel_src basicModel.cpp )
set ( basicModel_hdr basicModel.h )

add_executable (basicModel test.cpp ${basicModel_src} ${basicModel_hdr})



