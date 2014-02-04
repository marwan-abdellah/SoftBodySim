#ifndef __VERTEX_BUFFER_H
#define __VERTEX_BUFFER_H


class VertexBuffer {
public:
	enum {
		CPU_BUFFER,
		OPENGL_BUFFER,
	} type;
};

class CPUVertexBuffer {
public:
	CPUVertexBuffer() {}
	glm::vec3	*m_vertexPointer;
};

class GLVertexBuffer {
public:
	GLVertexBuffer() {}

	GLint getVBO(void) const;
};

#endif
