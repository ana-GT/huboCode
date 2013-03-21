# CMakeLists.txt for hubomz/mzcommon

# Create library mzcommon
set(mzcommon_srcs  TinyDom.cpp  TriMesh3.cpp  glstuff.cpp )
add_library( mzcommon ${mzcommon_srcs} )

set_target_properties( mzcommon PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

