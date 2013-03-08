cmake_minimum_required(VERSION 2.8)

project(dart_Lite)
message(STATUS "Generate makefile/project file for DART_Lite")

# Setting optimization flags
if(MSVC)
  message(STATUS "Setup Visual Studio Specific Flags")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Gy /W1 /EHsc")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
elseif(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
  set(CMAKE_CXX_FLAGS_PROFILE "-g -pg" CACHE STRING "gprof flags")
  set(CMAKE_CXX_FLAGS_OPROFILE "-g -fno-omit-frame-pointer" CACHE STRING "OProfile flags")
  set(CMAKE_CXX_FLAGS_DEBUG "-g" CACHE STRING "Debug flags")
  option(ENABLE_MSSE2 "Enable -msse2 flag" ON)
else()
  message(SEND_ERROR "Compiler not supported.")
endif()

# Find Eigen3
find_path(EIGEN3_INCLUDEDIR 
        NAMES Eigen/Core 
        PATH_SUFFIXES eigen3 eigen)

if( EIGEN3_INCLUDEDIR_NOTFOUND )
	message( STATUS "Eigen3 not found!")
else()
	message( STATUS "Eigen3 found, adding it to include directories")
	include_directories( ${EIGEN3_INCLUDEDIR} )
endif()

# Find TinyXML 
find_path(TINYXML_INCLUDEDIR 
	NAMES tinyxml.h
        PATH_SUFFIXES tinyxml)

if( TINYXML_INCLUDEDIR_NOTFOUND )
	message( STATUS "TinyXML not found!")
else()
	message( STATUS "TinyXML found, adding it to include directories")
	include_directories( ${TINYXML_INCLUDEDIR} )

endif()
set(TINYXML_LIBRARIES tinyxml)

# Include all the src files
include_directories(BEFORE src)
link_directories( ${CMAKE_SOURCE_DIR}/lib )


# Adding source files for library Dart-Lite
add_subdirectory(src)
add_subdirectory(test)

