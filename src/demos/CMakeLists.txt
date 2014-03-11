include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/src/)

add_executable(Demo Demo.cpp)

target_link_libraries(Demo glut-utils)	
target_link_libraries(Demo softbody-glrenderer)	
target_link_libraries(Demo softbody-engine)	
