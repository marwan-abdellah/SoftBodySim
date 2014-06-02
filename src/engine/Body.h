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
	const glm::mat4 &GetModelMatrix(void) { return mModelMatrix; }
	void SetModelMatrix(glm::mat4 &model) { mModelMatrix = model; }

protected:
	VertexBuffer				*mVertexes;
    ElementBuffer               *mEdges;
    ElementBuffer               *mFaces;
	glm::vec3					mColor;
	MeshData                    *mMesh;
	glm::mat4                   mModelMatrix;
};

#endif
