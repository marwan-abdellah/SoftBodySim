#include "Body.h"
#include "common.h"

using namespace glm;

Body::Body(MeshData *mesh) :
	mColor(glm::vec3(1.0,1.0,1.0))
{
	bool haveN = mesh->normals.size() == mesh->vertexes.size();
	bool haveT = mesh->textureCoords.size() == mesh->vertexes.size();
	mVertexes = new VertexBuffer(mesh->vertexes.size(), VertexBuffer::DYNAMIC,
			haveN, haveT);
	mVertexes->SetVertexes(mesh->vertexes);
	if (haveN) mVertexes->SetNormals(mesh->normals);
	if (haveT) mVertexes->SetTextureCoords(mesh->textureCoords);

	mFaces = new ElementBuffer(mesh->faces);
	mEdges = new ElementBuffer(mesh->edges);
	mMesh = mesh;
	mModelMatrix = mat4(1.0f);
}

Body::~Body(void)
{
	if (mVertexes) delete mVertexes;
	if (mFaces) delete mFaces;
	if (mEdges) delete mEdges;
}
