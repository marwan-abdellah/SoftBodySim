INCLUDE_DIRECTORIES(utils)

add_executable(Demo Demo.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/src/engine/include/)

add_subdirectory(utils)
add_subdirectory(engine)
add_subdirectory(renderer)

target_link_libraries(Demo utils)	
target_link_libraries(Demo softbody-glrenderer)	
