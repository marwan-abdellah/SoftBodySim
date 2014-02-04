add_library(utils SHARED GLUTApplication.cpp)

find_library(GLUT_LIBRARY glut)

target_link_libraries(utils ${GLUT_LIBRARY})
