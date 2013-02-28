cmake_minimum_required (VERSION 2.6)
project (VisualizationTab)

message (STATUS "Creating Makefile for VisualizationTab")

if(WIN32)
  set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  message (STATUS "Setup Visual Studio Specific Flags")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_DEBUG_POSTFIX "d")
  set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
  set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
  set (CMAKE_INSTALL_PREFIX /usr )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3  -std=c++0x")
  set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags")
endif()

#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCOLLADA_DOM_SUPPORT141 -DCOLLADA_DOM_SUPPORT150 -DCOLLADA_DOM_DAEFLOAT_IS64 -DDOM_DYNAMIC ${Boost_CFLAGS} ${COLLADA_DOM_C_FLAGS_OTHER}")	
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCOLLADA_DOM_DAEFLOAT_IS64 -fpermissive")	

message( "Collada dom cxx flags: ${COLLADA_DOM_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)
find_package(GRIP REQUIRED)
find_package(COLLADA_DOM REQUIRED COMPONENTS 1.5)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories(${GRIP_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})

# Collada DOM stuff
include_directories(${COLLADA_DOM_INCLUDE_DIRS} )
link_directories(${COLLADA_DOM_LIBRARY_DIRS})
message("Collada dom include dirs: ${COLLADA_DOM_INCLUDE_DIRS}")
message("Collada dom lib dirs: ${COLLADA_DOM_LIBRARY_DIRS}")

# Search all header and source files
set ( huboPlus_src huboPlus.cpp huboPlusApp.cpp collada_parser/collada_parser.cpp collada_parser/model.cpp )
set ( huboPlus_hdr huboPlus.h collada_parser/collada_parser.h collada_parser/model.h )

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
    add_executable (huboPlus WIN32 ${VisualizationTab_src} ${huboPlus_hdr})
    set_target_properties (huboPlus Tab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (huboPlus ${huboPlus_src} ${huboPlus_hdr})
	set_target_properties(huboPlus PROPERTIES COMPILE_FLAGS "-DDOM_INCLUDE_LIBXML -DCOLLADA_DOM_DAEFLOAT_IS64 -DCOLLADA_DOM_SUPPORT150 ${Boost_CFLAGS}"
  LINK_FLAGS "")
  endif()

  target_link_libraries (huboPlus ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${COLLADA_DOM_LIBRARIES} ${wxWidgets_LIBRARIES}  ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)



