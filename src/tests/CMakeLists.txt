include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/src/)
include_directories(.)

add_executable(diagonalize diagonalize.cpp)

target_link_libraries(diagonalize softbody-engine)	
