#ifndef VERTEX_BUFFER_H
#define VERTEX_BUFFER_H

#include "engine/geometry/Arrays.h"
#include "engine/model/MeshData.h"

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

/**
 *
 */
enum DrawType {
	POINTS,
	LINES,
	TRIANGLES,
	QUADS
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
	enum Usage {
		STATIC,
		DYNAMIC
	};
    VertexBuffer(size_t n, Usage usage, bool normals=false, bool textures=false);
    ~VertexBuffer(void);

    GLint GetVBO() const { return mVBO; }
	size_t GetVertexCount(void) const { return mVertexCount; }

	void SetVertexes(vec3Array_t &vertexes);
	void SetVertexes(glm::vec3 *vertexes);

	void SetNormals(vec3Array_t &vertexes);
	void SetNormals(glm::vec3 *vertexes);

	void SetTextureCoords(vec2Array_t &coords);
	void SetTextureCoords(glm::vec2 *coords);

    void Bind(int attr) const;
    void Unbind(void) const;

	void Draw(DrawType t) const;

	bool HaveTextureCoords(void) const { return mTextureOffset > 0; }
	bool HaveNormals(void) const { return mNormalsOffset > 0; }

private:
	void BindAttrs(int attrs) const;
	size_t mVertexCount;
    GLuint mVAO, mVBO;
	long mNormalsOffset, mTextureOffset;
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
