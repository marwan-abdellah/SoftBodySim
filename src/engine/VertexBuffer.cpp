#include "VertexBuffer.h"
#include <cstring>
#include "common.h"

using namespace glm;


GLVertexBuffer::GLVertexBuffer(unsigned int size) :
    VertexBuffer(size, OPENGL_BUFFER)
{
    memset(mVBO, 0x0, sizeof(GLint) * VERTEX_ATTR_LAST);
}

GLVertexBuffer::~GLVertexBuffer(void)
{
    glDeleteBuffers(VERTEX_ATTR_LAST, mVBO);
}

void GLVertexBuffer::setVertexes(glm::vec3 *data)
{
    if (mVBO[VERTEX_ATTR_POSITION] == 0)
        glGenBuffers(1, &mVBO[VERTEX_ATTR_POSITION]);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO[VERTEX_ATTR_POSITION]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * mVertexesCount, data, GL_STREAM_DRAW);
}

void GLVertexBuffer::setNormals(glm::vec3 *data)
{
    if (mVBO[VERTEX_ATTR_NORMAL] == 0)
        glGenBuffers(1, &mVBO[VERTEX_ATTR_NORMAL]);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO[VERTEX_ATTR_NORMAL]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * mVertexesCount, data, GL_DYNAMIC_DRAW);
}

void GLVertexBuffer::setTextCoords(glm::vec2 *data)
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

bool GLVertexBuffer::bind(VertexAttribute attr) const
{
    if (!mVBO[attr]) return false;

    switch(attr) {
        case VERTEX_ATTR_POSITION:
        case VERTEX_ATTR_NORMAL:
        case VERTEX_ATTR_COLOR:
            glBindBuffer(GL_ARRAY_BUFFER, mVBO[attr]);
            glEnableVertexAttribArray(attr);
            glVertexAttribPointer(attr, 3, GL_FLOAT, GL_FALSE, 0, 0);
            break;
        case VERTEX_ATTR_TEX_COORDS:
            glBindBuffer(GL_ARRAY_BUFFER, mVBO[attr]);
            glEnableVertexAttribArray(attr);
            glVertexAttribPointer(attr, 2, GL_FLOAT, GL_FALSE, 0, 0);
            break;
        default:
            return false;
    };
    return true;
}

void GLVertexBuffer::unbind(void) const
{
    glBindVertexArray(0);
    glDisableVertexAttribArray(VERTEX_ATTR_POSITION);
    glDisableVertexAttribArray(VERTEX_ATTR_NORMAL);
    glDisableVertexAttribArray(VERTEX_ATTR_COLOR);
    glDisableVertexAttribArray(VERTEX_ATTR_TEX_COORDS);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

GLElementBuffer::GLElementBuffer(unsigned int size, ElementDataType d)
    : ElementBuffer(size, OPENGL_BUFFER, d)
{
    glGenBuffers(1, &mBuffer);
}

void GLElementBuffer::setIndexes2(uvec2 *idxes)
{
    if (mDataType != EDGES) {
        ERR("Invalid data type. Expected uvec3");
        return;
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mElementsCount * sizeof(uvec2),
                 idxes, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void GLElementBuffer::setIndexes3(uvec3 *idxes)
{
    if (mDataType != TRIANGLES) {
        ERR("Invalid data type. Expected uvec2");
        return;
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mElementsCount * sizeof(uvec3),
                 idxes, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void GLElementBuffer::draw(void) const
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mBuffer);
    switch (mDataType) {
        case TRIANGLES:
            glDrawElements(GL_TRIANGLES, mElementsCount * 3, GL_UNSIGNED_INT, 0);
            break;
        case EDGES:
            glDrawElements(GL_LINES, mElementsCount * 2, GL_UNSIGNED_INT, 0);
    };
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

GLElementBuffer::~GLElementBuffer(void)
{
    glDeleteBuffers(1, &mBuffer);
}

GLElementBuffer::GLElementBuffer(index2Array_t &array)
	:GLElementBuffer(array.size(), ElementBuffer::EDGES)
{
	setIndexes2(&array[0]);
}

GLElementBuffer::GLElementBuffer(index3Array_t &array)
	:GLElementBuffer(array.size(), ElementBuffer::TRIANGLES)
{
	setIndexes3(&array[0]);
}
