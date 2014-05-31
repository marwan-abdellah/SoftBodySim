#include "Body.h"

Body::Body(MeshData *mesh) :
	mColor(glm::vec3(1,1,1))
{
	mVertexes = new VertexBuffer(mesh->vertexes, mesh->normals, mesh->textureCoords);
	mFaces = new ElementBuffer(mesh->faces);
	mEdges = new ElementBuffer(mesh->edges);
	mMesh = mesh;
}

Body::~Body(void)
{
	if (mVertexes) delete mVertexes;
	if (mFaces) delete mFaces;
	if (mEdges) delete mEdges;
}
