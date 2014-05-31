#ifndef SB_BODY_H
#define SB_BODY_H

#include "geometry/Sphere.h"
#include "model/MeshData.h"
#include "VertexBuffer.h"


/**
 * @class Abstract class 
 */
class Body {
public:
	Body(MeshData *md);
	virtual ~Body();

    VertexBuffer *GetVertexes(void) { return mVertexes; }
	ElementBuffer *getEdges(void) { return mEdges; }
	ElementBuffer *getFaces(void) { return mFaces; }
	void SetColor(glm::vec3 &color) { mColor = color; }
	void SetColor(glm::vec3 color) { mColor = color; }
	const glm::vec3 GetColor(void) { return mColor; }
	const MeshData *GetMesh(void) { return mMesh; }

protected:
	VertexBuffer				*mVertexes;
    ElementBuffer               *mEdges;
    ElementBuffer               *mFaces;
	glm::vec3					mColor;
	MeshData                    *mMesh;
};

#endif
