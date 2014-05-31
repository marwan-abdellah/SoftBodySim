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
    };
    VertexBuffer(vec3Array_t &vertexes);
    VertexBuffer(vec3Array_t &vertexes, vec2Array_t &textures);
    VertexBuffer(vec3Array_t &vertexes, vec3Array_t &normals);
    VertexBuffer(vec3Array_t &vertexes, vec3Array_t &normals, vec2Array_t &tex);
    ~VertexBuffer(void);

    GLint GetVBO() const { return mVBO; }
	size_t GetVertexCount(void) const { return mVertexCount; }

	void SetVertexes(vec3Array_t &vertexes);
	void SetVertexes(glm::vec3 *vertexes);
	void SetNormals(vec3Array_t &vertexes);
	void SetTextureCoords(vec2Array_t &vertexes);

    void Bind(int attr) const;
    void Unbind(void) const;

	void Draw(DrawType t) const;

private:
	void BindAttrs(int attrs) const;
	size_t mVertexCount;
    GLuint mVAO, mVBO;
	int mNormalsOffset, mTextureOffset;
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
