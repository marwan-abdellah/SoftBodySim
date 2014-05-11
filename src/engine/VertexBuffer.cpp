#include "VertexBuffer.h"
#include <cstring>
#include <cstddef>
#include "common.h"

using namespace glm;


VertexBuffer::VertexBuffer(vertexArray_t &vertexes)
{
	glGenVertexArrays(1, &mVAO);
	glBindVertexArray(mVAO);
	glGenBuffers(1, &mVBO);
	SetData(vertexes);

	glEnableVertexAttribArray(VERTEX_ATTR_POSITION);
	glVertexAttribPointer(VERTEX_ATTR_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), NULL);
	glEnableVertexAttribArray(VERTEX_ATTR_NORMAL);
	glVertexAttribPointer(VERTEX_ATTR_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, normal)));
	glEnableVertexAttribArray(VERTEX_ATTR_TEX_COORDS);
	glVertexAttribPointer(VERTEX_ATTR_TEX_COORDS, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, texture)));

	glBindVertexArray(0);
}

VertexBuffer::~VertexBuffer(void)
{
    glDeleteBuffers(1, &mVAO);
    glDeleteBuffers(1, &mVBO);
}

void VertexBuffer::SetData(vertexArray_t &vertexes)
{
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	mVertexCount = vertexes.size();
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * mVertexCount, &vertexes[0], GL_STATIC_DRAW);
}

void VertexBuffer::Draw(DrawType t) const
{
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	GLenum type;
	switch (t) {
		case TRIANGLES:
			type = GL_TRIANGLES;
			break;
		case LINES:
			type = GL_LINES;
			break;
	}
	glDrawArrays(type, 0, mVertexCount);
}

void VertexBuffer::Bind(int attrs) const
{
	glBindVertexArray(mVAO);
}

void VertexBuffer::Unbind(void) const
{
    glBindVertexArray(0);
    glDisableVertexAttribArray(VERTEX_ATTR_POSITION);
    glDisableVertexAttribArray(VERTEX_ATTR_NORMAL);
    glDisableVertexAttribArray(VERTEX_ATTR_COLOR);
    glDisableVertexAttribArray(VERTEX_ATTR_TEX_COORDS);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

ElementBuffer::ElementBuffer(index2Array_t &array)
{
    glGenBuffers(1, &mBuffer);
	setData(array);
}

ElementBuffer::ElementBuffer(index3Array_t &array)
{
    glGenBuffers(1, &mBuffer);
	setData(array);
}

ElementBuffer::~ElementBuffer(void)
{
    glDeleteBuffers(1, &mBuffer);
}

void ElementBuffer::setData(index2Array_t &array)
{
	mElementsCount = array.size();
	mDataType = LINES;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mElementsCount * sizeof(uvec2),
                 &array[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void ElementBuffer::setData(index3Array_t &array)
{
	mElementsCount = array.size();
	mDataType = TRIANGLES;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mElementsCount * sizeof(uvec3),
                 &array[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void ElementBuffer::Draw(void) const
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBuffer);
    switch (mDataType) {
        case TRIANGLES:
            glDrawElements(GL_TRIANGLES, mElementsCount * 3, GL_UNSIGNED_INT, 0);
            break;
        case LINES:
            glDrawElements(GL_LINES, mElementsCount * 2, GL_UNSIGNED_INT, 0);
    };
}
