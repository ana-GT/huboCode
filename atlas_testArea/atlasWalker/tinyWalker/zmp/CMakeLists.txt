# atlasZmp Library
add_library( atlaszmp zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp ) 
set_target_properties( atlaszmp PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
#set(zmpdemo_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)


