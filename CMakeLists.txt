cmake_minimum_required(VERSION 2.8)

project(SoftBodySim)

include_directories(src)

add_definitions(-g)
add_definitions(-Wall)

#find_package(CUDA REQUIRED)

#list(APPEND CUDA_NVCC_FLAGS "-g")
#list(APPEND CUDA_NVCC_FLAGS "-G")

add_subdirectory(src bin)
#add_subdirectory(lib lib)

