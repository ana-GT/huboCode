# CMakeLists.txt for hubomz/mzcommon

set(mzcommon_srcs
  mersenne.c
  gauss.c
  GlCamera.cpp
  MzGlutApp.cpp
  TinyDom.cpp
  TriMesh3.cpp
  glstuff.cpp)

add_library( mzcommon ${mzcommon_srcs} )


