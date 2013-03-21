# CMakeLists.txt for hubomz
cmake_minimum_required(VERSION 2.8)
project(zmpTab)

# To find libraries
include(FindPkgConfig)

# Eigen
pkg_search_module(EIGEN3 REQUIRED eigen3>=3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories( . )

# Add subdirectories (libraries)
add_subdirectory(mzcommon)
add_subdirectory(hubomz)

# Flags 
if(WIN32)
  set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  message (STATUS "Setup Visual Studio Specific Flags")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc /openmp")
  set (CMAKE_DEBUG_POSTFIX "d")
  set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
  set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
  set (CMAKE_INSTALL_PREFIX /usr )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -g  -std=c++0x")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3  -std=c++0x")
endif()

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


set ( zmp_src planningTab.cpp planningTabApp.cpp Controller.cpp zmp/zmpdemo.cpp )
set ( zmp_hdr planningTab.h Controller.h )

link_directories( ${CMAKE_SOURCE_DIR}/lib )

if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)

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

  target_link_libraries (zmpTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}  ${Boost_LIBRARIES} mzcommon hubomz expat)

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)





