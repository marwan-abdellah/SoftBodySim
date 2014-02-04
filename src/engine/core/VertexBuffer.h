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

	VertexBuffer(unsigned int s, VertexBufferType type = CPU_BUFFER) : mType(type), mVertexesCount(s) {}
	VertexBufferType getType(void) { return mType; }

	virtual void setVertexes(glm::vec3 *data) = 0;
	virtual void setNormals(glm::vec3 *data) = 0;
	virtual void setTextCoords(glm::uvec2 *data) = 0;
	virtual void setColors(glm::vec3 *data) = 0;

protected:
	VertexBufferType mType;
	unsigned int mVertexesCount;
};

class GLVertexBuffer : public VertexBuffer {
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
