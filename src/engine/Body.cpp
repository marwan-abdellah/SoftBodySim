#include "Body.h"

Body::Body(MeshData *mesh)
{
	mVertexes = new VertexBuffer(mesh->vertexes);
	mFaces = new ElementBuffer(mesh->faces);
	mEdges = new ElementBuffer(mesh->edges);
}

Body::~Body(void)
{
	if (mVertexes) delete mVertexes;
	if (mFaces) delete mFaces;
	if (mEdges) delete mEdges;
}
