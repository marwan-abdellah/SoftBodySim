#include "VertexBuffer.h"
#include <cstring>
#include <cstddef>
#include "common.h"

using namespace glm;

VertexBuffer::VertexBuffer(size_t n, Usage usage, bool normals, bool textures) :
	mVertexCount(n),
	mNormalsOffset(0),
	mTextureOffset(0)
{
	glGenVertexArrays(1, &mVAO);
	glBindVertexArray(mVAO);
	glGenBuffers(1, &mVBO);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);

	size_t size = sizeof(vec3) * mVertexCount;

	glEnableVertexAttribArray(VERTEX_ATTR_POSITION);
	glVertexAttribPointer(VERTEX_ATTR_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), 0);
	
	if (normals) {
		mNormalsOffset = sizeof(vec3) * mVertexCount;
		size += sizeof(vec3) * mVertexCount;
		glEnableVertexAttribArray(VERTEX_ATTR_NORMAL);
		glVertexAttribPointer(VERTEX_ATTR_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)mNormalsOffset);
	}
	if (textures) {
		mTextureOffset = mNormalsOffset + sizeof(vec3) * mVertexCount;
		size += sizeof(vec2) * mVertexCount;
		glEnableVertexAttribArray(VERTEX_ATTR_TEX_COORDS);
		glVertexAttribPointer(VERTEX_ATTR_TEX_COORDS, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)mTextureOffset);
	}

	GLenum type = usage == STATIC ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW;
	glBufferData(GL_ARRAY_BUFFER, size , 0, type);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

VertexBuffer::~VertexBuffer(void)
{
    glDeleteBuffers(1, &mVBO);
    glDeleteVertexArrays(1, &mVAO);
}

void VertexBuffer::SetNormals(vec3Array_t &vertexes)
{
	if (!mNormalsOffset) return;
	if (mVertexCount != vertexes.size()) {
		ERR("Invalid normal array size. Should match vertex array.");
		return;
	}
	glBindVertexArray(mVAO);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glBufferSubData(GL_ARRAY_BUFFER, mNormalsOffset, mVertexCount * sizeof(vec3), &vertexes[0]);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void VertexBuffer::SetVertexes(glm::vec3 *vertexes)
{
	glBindVertexArray(mVAO);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, mVertexCount * sizeof(vec3), vertexes);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void VertexBuffer::SetVertexes(vec3Array_t &vertexes)
{
	if (mVertexCount != vertexes.size()) {
		ERR("Invalid normal array size. Should match vertex array.");
		return;
	}
	glBindVertexArray(mVAO);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, mVertexCount * sizeof(vec3), &vertexes[0]);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void VertexBuffer::SetTextureCoords(vec2Array_t &coords)
{
	if (!mTextureOffset) return;
	if (mVertexCount != coords.size()) {
		ERR("Invalid texture coords array size. Should match vertex array.");
		return;
	}
	glBindVertexArray(mVAO);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glBufferSubData(GL_ARRAY_BUFFER, mTextureOffset, mVertexCount * sizeof(vec2), &coords[0]);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void VertexBuffer::Draw(DrawType t) const
{
	glBindVertexArray(mVAO);
	GLenum type;
	switch (t) {
		case TRIANGLES:
			type = GL_TRIANGLES;
			break;
		case LINES:
			type = GL_LINES;
			break;
		case QUADS:
			type = GL_QUADS;
			break;
	}
	glDrawArrays(type, 0, mVertexCount);
	glBindVertexArray(0);
}

void VertexBuffer::Bind(int attrs) const
{
	glBindVertexArray(mVAO);
}

void VertexBuffer::Unbind(void) const
{
    glBindVertexArray(0);
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
			break;
        case QUADS:
            glDrawElements(GL_QUADS, mElementsCount * 4, GL_UNSIGNED_INT, 0);
			break;
    };
}
