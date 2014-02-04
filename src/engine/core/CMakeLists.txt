set (SRCS
		SoftBody.cpp
		CUDASoftBodySolver.cpp
	)

find_package(CUDA REQUIRED)

list(APPEND CUDA_NVCC_FLAGS "-g")
list(APPEND CUDA_NVCC_FLAGS "-G")

include_directories('.')
include(FindCUDA)

cuda_add_library(softbody-engine SHARED ${SRCS})
