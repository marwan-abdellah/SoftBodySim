#ifndef __VERTEX_BUFFER_H
#define __VERTEX_BUFFER_H

#include <glm/glm.hpp>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

class VertexBuffer {
public:
	enum VertexBufferType {
		CPU_BUFFER,
		OPENGL_BUFFER,
	};

	VertexBuffer(unsigned int s, VertexBufferType type = CPU_BUFFER) : m_type(type), mVertexesCount(s) {}
	VertexBufferType getType(void) { return m_type; }
protected:
	VertexBufferType m_type;
	glm::vec3				*mVertexes;
	glm::vec3				*mNormals;
	glm::vec2				*mTexCoords;
	glm::vec3				*mColors;
	unsigned int mVertexesCount;
};

class GLVertexBuffer : VertexBuffer {
public:
	GLVertexBuffer(unsigned int size);

	void setVertexes(glm::vec3 *data);
	void setNormals(glm::vec3 *data);
	void setTextCoords(glm::uvec2 *data);
	void setColors(glm::vec3 *data);

	enum VertexAttribute {
		VERTEX_ATTR_POSITION,
		VERTEX_ATTR_NORMAL,
		VERTEX_ATTR_TEX_COORDS,
		VERTEX_ATTR_COLOR,
		VERTEX_ATTR_LAST
	};

	bool bind(VertexAttribute attr);
	void unbind(void);

private:
	GLuint mVBO[VERTEX_ATTR_LAST];
};

#endif
