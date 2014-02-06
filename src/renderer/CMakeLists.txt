set (SRCS
		Shader.cpp
	)

include_directories(.)
include_directories(${CMAKE_SOURCE_DIR}/src/engine/include/)

find_library(OPENGL_LIBRARY GL)

add_library(softbody-glrenderer SHARED ${SRCS})
target_link_libraries(softbody-glrenderer ${OPENGL_LIBRARY})

