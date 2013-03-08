project(dart_Lite)

# Search all header and source files
file(GLOB_RECURSE dart_Lite_srcs "*.cpp")
file(GLOB_RECURSE dart_Lite_hdrs "*.h")

# Setup directory structure for Visual Studio
set(SUBDIRS dynamics kinematics robotics utils)
foreach(SUBDIR ${SUBDIRS})
	file(GLOB_RECURSE dir_files ${SUBDIR}/*.h ${SUBDIR}/*.cpp)
	source_group(${SUBDIR} FILES ${dir_files})
endforeach()

# Library 
add_library(dart_Lite ${dart_Lite_srcs} ${dart_Lite_hdrs})
#target_link_libraries(dart_Lite ${DARTExt_LIBRARIES})
set_target_properties(dart_Lite PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
    set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

#install(DIRECTORY ./ DESTINATION include/dart 
#        FILES_MATCHING PATTERN "*.h*" 
#        PATTERN "*CMake*" EXCLUDE)
        
#install(TARGETS dart DESTINATION lib)
