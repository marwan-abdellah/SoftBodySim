INCLUDE_DIRECTORIES(utils)

add_executable(Demo Demo.cpp)

add_subdirectory(utils)
add_subdirectory(engine)

target_link_libraries(Demo utils)	
