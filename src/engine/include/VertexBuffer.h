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
    VertexBufferType getType(void) const { return mType; }

    virtual void setVertexes(glm::vec3 *data) = 0;
    virtual void setNormals(glm::vec3 *data) = 0;
    virtual void setTextCoords(glm::vec2 *data) = 0;
    virtual void setColors(glm::vec3 *data) = 0;

protected:
    VertexBufferType mType;
    unsigned int mVertexesCount;
};

class GLVertexBuffer : public VertexBuffer {
public:
    GLVertexBuffer(unsigned int size);
    ~GLVertexBuffer(void);

    void setVertexes(glm::vec3 *data);
    void setNormals(glm::vec3 *data);
    void setTextCoords(glm::vec2 *data);
    void setColors(glm::vec3 *data);

    enum VertexAttribute {
        VERTEX_ATTR_POSITION,
        VERTEX_ATTR_NORMAL,
        VERTEX_ATTR_TEX_COORDS,
        VERTEX_ATTR_COLOR,
        VERTEX_ATTR_LAST
    };

    GLint getVBO(VertexAttribute a) const { return mVBO[a]; }

    bool bind(VertexAttribute attr) const;
    void unbind(void) const;

private:
    GLuint mVBO[VERTEX_ATTR_LAST];
};

class ElementBuffer {
public:
    enum ElementBufferType {
        CPU_BUFFER,
        OPENGL_BUFFER,
    };

    enum ElementDataType {
        TRIANGLES,
        EDGES
    };

    ElementBuffer(unsigned int size, ElementBufferType t, ElementDataType d)
        : mElementsCount(size), mType(t), mDataType(d) {}

    ElementDataType getDataType(void) { return mDataType;}
    ElementBufferType getType(void) { return mType;}
protected:
    unsigned int mElementsCount;
    ElementBufferType mType;
    ElementDataType mDataType;
};

class GLElementBuffer : public ElementBuffer {
public:
    GLElementBuffer(unsigned int size, ElementDataType d);
    ~GLElementBuffer(void);
    void draw(void) const;
    void setIndexes2(glm::uvec2 *idx);
    void setIndexes3(glm::uvec3 *idx);
private:
    GLuint mBuffer;
};

#endif
