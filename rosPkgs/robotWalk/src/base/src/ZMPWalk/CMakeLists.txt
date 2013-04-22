# ******************************
# CMake for PreviewController
# ******************************

cmake_minimum_required(VERSION 2.8)
project(PreviewController)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_BUILD_TYPE Debug)

message(STATUS "Creating Makefile for PreviewController")

include_directories( . )
include_directories( /usr/local/include/eigen3/ )

set ( PreviewController_src PreviewController.cpp )
set ( PreviewController_hdr PreviewController.h )

add_executable ( PreviewController PreviewControllerTest.cpp ${PreviewController_src} ${PreviewController_hdr} )
