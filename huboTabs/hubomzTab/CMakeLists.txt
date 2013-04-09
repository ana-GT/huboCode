# CMakeLists.txt for hubomz
cmake_minimum_required(VERSION 2.6)

project(HUBOMZ)

include(mzsettings.cmake)

include_directories( . )

add_subdirectory(mzcommon)
add_subdirectory(src)
add_subdirectory(zmp)

# Flags (Linux)
set (CMAKE_INSTALL_PREFIX /usr )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -g  -std=c++0x")


# Find Dart and Grip
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)
find_package(GRIP REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories(${GRIP_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})

# Find expat needed for Matt's parsing
find_library( EXPAT_LIBRARY expat )

# Source files
set ( zmp_src planningTab.cpp planningTabApp.cpp Controller.cpp  )
set ( zmp_hdr planningTab.h Controller.h )

link_directories( ${CMAKE_BINARY_DIR}/lib )

set (wxWidgets_USE_LIBS base core gl)


find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
    add_executable (zmpTab WIN32 ${zmp_src} ${zmp_hdr})
    set_target_properties (zmpTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (zmpTab ${zmp_src} ${zmp_hdr})
  endif()
  
  target_link_libraries (zmpTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}  ${Boost_LIBRARIES} hubozmp   hubomz mzcommon expat)
  
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

