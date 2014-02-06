include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${CMAKE_SOURCE_DIR}/src/utils/)
include_directories(${CMAKE_SOURCE_DIR}/src/renderer/include)
include_directories(${CMAKE_SOURCE_DIR}/src/engine/include)

add_executable(Demo Demo.cpp)

target_link_libraries(Demo utils)	
target_link_libraries(Demo softbody-glrenderer)	
target_link_libraries(Demo softbody-engine)	
