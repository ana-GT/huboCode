cmake_minimum_required (VERSION 2.8)
project (testFK)

message (STATUS "Creating Makefile for testFK")

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


# Search all header and source files
set ( testFK_src testFK.cpp motion_rt.cpp )
#set ( testFK_hdr testFK.h )


  if(MSVC)
    add_executable (testFK WIN32 ${testFK_src} )
    set_target_properties (testFK Tab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (testFK ${testFK_src} )
    #set_target_properties(testFK PROPERTIES COMPILE_FLAGS "" LINK_FLAGS "")
  endif()

  target_link_libraries (testFK dart_Lite tinyxml)
  set_target_properties(testFK PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


  if(MSVC)
    add_executable (testFK_transform WIN32 testFK_transform.cpp motion_rt.cpp )
    set_target_properties (testFK_transform Tab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (testFK_transform testFK_transform.cpp motion_rt.cpp )
    #set_target_properties(testFK PROPERTIES COMPILE_FLAGS "" LINK_FLAGS "")
  endif()

  target_link_libraries (testFK_transform dart_Lite tinyxml)
  set_target_properties(testFK_transform PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


