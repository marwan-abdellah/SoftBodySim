include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/src/)
include_directories(.)

find_library(GLFW_LIBRARY glfw)
find_package(CUDA QUIET)

if (CUDA_FOUND)
	add_definitions(-DENABLE_CUDA=1)
endif ()

add_executable(Demo Demo.cpp GLFWApplication.cpp)

target_link_libraries(Demo softbody-engine ${GLFW_LIBRARY})
