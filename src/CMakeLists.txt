INCLUDE_DIRECTORIES(utils)

add_executable(Demo Demo.cpp)

add_subdirectory(utils)

target_link_libraries(Demo utils)	
