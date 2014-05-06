set (SRCS
		SoftBody.cpp
		CUDASoftBodySolver.cu
		CUDASoftBodySolverKernel.cu
		VertexBuffer.cpp
		MeshData.cpp
	)

find_package(CUDA REQUIRED)

list(APPEND CUDA_NVCC_FLAGS "-g")
list(APPEND CUDA_NVCC_FLAGS "-G")
list(APPEND CUDA_NVCC_FLAGS "-arch sm_30")

include_directories(.)
include_directories(${CMAKE_SOURCE_DIR}/src/)
include(FindCUDA)

find_library(OPENGL_LIBRARY GL)

cuda_add_library(softbody-engine SHARED ${SRCS})
target_link_libraries(softbody-engine ${OPENGL_LIBRARY})

add_subdirectory(geometry)
