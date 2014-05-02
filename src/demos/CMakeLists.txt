include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/src/)
include_directories(.)

add_executable(Demo Demo.cpp GLFWApplication.cpp)

target_link_libraries(Demo softbody-glrenderer)	
target_link_libraries(Demo softbody-engine)	

find_library(GLFW_LIBRARY glfw)

target_link_libraries(Demo ${GLFW_LIBRARY})
