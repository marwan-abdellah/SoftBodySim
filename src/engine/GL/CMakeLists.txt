set (SRCS
		Shader.cpp
	)

include_directories('.', '../core')

find_library(OPENGL_LIBRARY GL)

add_library(softbody-glrenderer SHARED ${SRCS})
target_link_libraries(softbody-glrenderer ${OPENGL_LIBRARY})

