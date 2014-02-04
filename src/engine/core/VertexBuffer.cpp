#include "VertexBuffer.h"
#include <cstring>

using namespace glm;


GLVertexBuffer::GLVertexBuffer(unsigned int size) :
	VertexBuffer(size, OPENGL_BUFFER)
{
	memset(mVBO, 0x0, sizeof(GLint) * VERTEX_ATTR_LAST);
}

void GLVertexBuffer::setVertexes(glm::vec3 *data)
{
	if (mVBO[VERTEX_ATTR_POSITION] == 0)
		glGenBuffers(1, &mVBO[VERTEX_ATTR_POSITION]);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO[VERTEX_ATTR_POSITION]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * mVertexesCount, data, GL_DYNAMIC_DRAW);
}

void GLVertexBuffer::setNormals(glm::vec3 *data)
{
	if (mVBO[VERTEX_ATTR_NORMAL] == 0)
		glGenBuffers(1, &mVBO[VERTEX_ATTR_NORMAL]);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO[VERTEX_ATTR_NORMAL]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * mVertexesCount, data, GL_DYNAMIC_DRAW);
}

void GLVertexBuffer::setTextCoords(glm::uvec2 *data)
{
	if (mVBO[VERTEX_ATTR_TEX_COORDS] == 0)
		glGenBuffers(1, &mVBO[VERTEX_ATTR_TEX_COORDS]);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO[VERTEX_ATTR_TEX_COORDS]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(uvec2) * mVertexesCount, data, GL_DYNAMIC_DRAW);
}

void GLVertexBuffer::setColors(glm::vec3 *data)
{
	if (mVBO[VERTEX_ATTR_COLOR] == 0)
		glGenBuffers(1, &mVBO[VERTEX_ATTR_COLOR]);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO[VERTEX_ATTR_COLOR]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * mVertexesCount, data, GL_DYNAMIC_DRAW);
}

bool GLVertexBuffer::bind(VertexAttribute attr)
{
	if (!mVBO[attr]) return false;

	switch(attr) {
		case VERTEX_ATTR_POSITION:
		case VERTEX_ATTR_NORMAL:
		case VERTEX_ATTR_COLOR:
			glEnableVertexAttribArray(attr);
			glBindBuffer(GL_ARRAY_BUFFER, mVBO[attr]);
			glVertexAttribPointer(attr, 3, GL_FLOAT, GL_FALSE, 0, 0);
			break;
		case VERTEX_ATTR_TEX_COORDS:
			glEnableVertexAttribArray(attr);
			glBindBuffer(GL_ARRAY_BUFFER, mVBO[attr]);
			glVertexAttribPointer(attr, 2, GL_FLOAT, GL_FALSE, 0, 0);
			break;
		default:
			return false;
	};
	return true;
}

void GLVertexBuffer::unbind(void)
{
	glBindVertexArray(0);
	glDisableVertexAttribArray(VERTEX_ATTR_POSITION);
	glDisableVertexAttribArray(VERTEX_ATTR_NORMAL);
	glDisableVertexAttribArray(VERTEX_ATTR_COLOR);
	glDisableVertexAttribArray(VERTEX_ATTR_TEX_COORDS);
}

