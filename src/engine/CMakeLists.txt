set (COMMON_SRCS
		model/SoftBody.cpp
		model/MeshData.cpp
		model/OBJLexer.cpp
		model/Material.cpp
		solver/SoftBodySolver.cpp
		solver/CPUSoftBodySolver.cpp
		geometry/Math.cpp
		renderer/Shader.cpp
        renderer/Renderer.cpp
        renderer/Camera.cpp
		renderer/VertexBuffer.cpp
		renderer/Body.cpp
	)

set (GPU_SRCS
		CUDASoftBodySolver.cu
		CUDASoftBodySolverKernel.cu
	)

add_subdirectory(geometry)

find_package(CUDAU QUIET)
find_library(OPENGL_LIBRARY GL)

include_directories(.)
include_directories(${CMAKE_SOURCE_DIR}/src/)

if (CUDA_FOUND)
	MESSAGE(building softbody-engine w/o CUDA support.)
	list(APPEND CUDA_NVCC_FLAGS "-g")
	list(APPEND CUDA_NVCC_FLAGS "-G")
	list(APPEND CUDA_NVCC_FLAGS "-arch sm_30")
	include(FindCUDA)
	set (SRCS ${COMMON_SRCS} ${GPU_SRCS})
	cuda_add_library(softbody-engine SHARED ${SRCS})
else (CUDA_FOUND)
	add_definitions(-g)
	add_definitions(-Wall)
	add_definitions(-DENABLE_DEBUG=1)
	set (SRCS ${COMMON_SRCS})
	add_library(softbody-engine SHARED ${SRCS})
endif ()

target_link_libraries(softbody-engine ${OPENGL_LIBRARY})
