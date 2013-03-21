# CMakeLists.txt for hubomz/src
# Original from Matt Zucker (adapted to DART/GRIP)

# Create library hubomz
set( hubomz_src fakerave.cpp fakerave_parse.cpp HuboPlus.cpp HuboKin.cpp )
add_library( hubomz  ${hubomz_src} )
target_link_libraries( hubomz mzcommon )

set_target_properties( hubomz PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

# Add test executables
#add_executable( testkin testkin.cpp )
#target_link_libraries( testkin hubomz mzcommon ${OPENGL_LIBRARY} expat )

#add_executable( testzmp testzmp.cpp )
#target_link_libraries( testzmp hubomz mzcommon ${OPENGL_LIBRARY} expat )

#add_executable( testik testik.cpp )
#target_link_libraries( testik hubomz mzcommon ${OPENGL_LIBRARY} expat )

#add_executable( reconcile reconcile.cpp )
#target_link_libraries( reconcile hubomz mzcommon ${OPENGL_LIBRARY} expat )

#add_gui_app( walkdemo walkdemo.cpp )
#target_link_libraries( walkdemo hubomz mzcommon ${GLUT_LIBRARY} 
#  ${OPENGL_LIBRARY} expat )
