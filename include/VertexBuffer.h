#ifndef __VERTEX_BUFFER_H
#define __VERTEX_BUFFER_H


class VertexBuffer {
	enum {
		CPU_BUFFER,
		OPENGL_BUFFER,
	} type;

	glm::vec3	*m_vertexPointer;

};


#endif
