#include "Body.h"

using namespace glm;

Body::Body(MeshData *mesh) :
	mColor(glm::vec3(1,1,1))
{
	mVertexes = new VertexBuffer(mesh->vertexes, mesh->normals, mesh->textureCoords);

	mFaces = new ElementBuffer(mesh->faces);
	mEdges = new ElementBuffer(mesh->edges);
	mMesh = mesh;
	mModelMatrix = mat4(1.0f);
	mColor = vec3(1.0, 1.0, 1.0);
}

Body::~Body(void)
{
	if (mVertexes) delete mVertexes;
	if (mFaces) delete mFaces;
	if (mEdges) delete mEdges;
}
