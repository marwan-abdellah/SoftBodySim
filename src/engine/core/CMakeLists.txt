set (SRCS
		SoftBody.cpp
		CUDASoftBodySolver.cu
		CUDASoftBodySolver.cpp
		VertexBuffer.cpp
	)

find_package(CUDA REQUIRED)

list(APPEND CUDA_NVCC_FLAGS "-g")
list(APPEND CUDA_NVCC_FLAGS "-G")

include_directories(.)
include(FindCUDA)

find_library(OPENGL_LIBRARY GL)

cuda_add_library(softbody-engine SHARED ${SRCS})
target_link_libraries(softbody-engine ${OPENGL_LIBRARY})
