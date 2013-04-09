add_library(hubozmp zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp)

add_gui_app(zmpdemo zmpdemo.cpp)

set(zmpdemo_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)

target_link_libraries(zmpdemo ${zmpdemo_libs})

