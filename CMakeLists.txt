cmake_minimum_required(VERSION 2.8)

project(SoftBodySim)

include_directories(include)

add_definitions(-g)
add_definitions(-std=c++11)
add_definitions(-Wall)

add_subdirectory(src)
