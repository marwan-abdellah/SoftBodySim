#ifndef SB_ENGINE_BODY_H_
#define SB_ENGINE_BODY_H_

#include "sbs/geometry/Sphere.h"
#include "sbs/model/MeshData.h"
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
	const MeshData *GetMesh(void) { return mMesh; }
	const glm::mat4 &GetModelMatrix(void) { return mModelMatrix; }
	void SetModelMatrix(glm::mat4 &model) { mModelMatrix = model; }
	const Sphere &GetBoundingSphere(void) { return mBS; }
protected:
	VertexBuffer				*mVertexes;
    ElementBuffer               *mEdges;
    ElementBuffer               *mFaces;
	glm::vec3					mColor;
	MeshData                    *mMesh;
	glm::mat4                   mModelMatrix;
	Sphere                      mBS; // bounding sphere
};

#endif
