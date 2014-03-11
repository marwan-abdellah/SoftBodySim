add_library(glut-utils SHARED GLUTApplication.cpp)

find_library(GLUT_LIBRARY glut)

target_link_libraries(glut-utils ${GLUT_LIBRARY})
