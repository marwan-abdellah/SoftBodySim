set (SRCS
		Body.cpp
		SoftBody.cpp
		CUDASoftBodySolver.cu
		CUDASoftBodySolverKernel.cu
		VertexBuffer.cpp
		model/MeshData.cpp
		model/OBJLexer.cpp
		model/Material.cpp
		solver/SoftBodySolver.cpp
		solver/CPUSoftBodySolver.cpp
		geometry/Math.cpp
		renderer/Shader.cpp
        renderer/Renderer.cpp
        renderer/Camera.cpp
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
