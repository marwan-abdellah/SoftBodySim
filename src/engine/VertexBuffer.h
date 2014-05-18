#ifndef VERTEX_BUFFER_H
#define VERTEX_BUFFER_H

#include "geometry/Arrays.h"
#include "model/MeshData.h"

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

/**
 *
 */
enum DrawType {
	TRIANGLES,
	LINES 
};

/**
 * @brief Class wrapping OpenGL VBO and Vertex Attribute managing
 */
class VertexBuffer {
public:
    enum VertexAttribute {
        VERTEX_ATTR_POSITION,
        VERTEX_ATTR_TEX_COORDS,
        VERTEX_ATTR_NORMAL,
        VERTEX_ATTR_COLOR,
    };
    VertexBuffer(MeshData::vertexArray_t &vertexes);
    ~VertexBuffer(void);

	void SetData(MeshData::vertexArray_t &vertexes);
    GLint GetVBO() const { return mVBO; }

	size_t GetVertexCount(void) const { return mVertexCount; }

    void Bind(int attrs) const;
    void Unbind(void) const;

	void Draw(DrawType t) const;

private:
	void BindAttrs(int attrs) const;
	size_t mVertexCount;
    GLuint mVAO, mVBO;
};

class ElementBuffer {
public:
	ElementBuffer(index2Array_t &array);
	ElementBuffer(index3Array_t &array);
	~ElementBuffer();

    DrawType getDataType(void) { return mDataType;}

	void setData(index2Array_t &array);
	void setData(index3Array_t &array);

    void Draw(void) const;

private:
    GLuint mBuffer;
    size_t mElementsCount;
    DrawType mDataType;
};

#endif
