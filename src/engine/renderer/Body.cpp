#include "Body.h"
#include "common.h"

using namespace glm;

Body::Body(MeshData *mesh) :
	mColor(glm::vec3(1.0,1.0,1.0))
{
	bool haveN = mesh->GetNormals().size() > 0;
	bool haveT = mesh->GetTextureCoords().size() > 0;
	mVertexes = new VertexBuffer(mesh->GetVertexes().size(),
			VertexBuffer::DYNAMIC,
			haveN, haveT);
	mVertexes->SetVertexes(mesh->GetVertexes());
	if (haveN) mVertexes->SetNormals(mesh->GetNormals());
	if (haveT) mVertexes->SetTextureCoords(mesh->GetTextureCoords());

	mFaces = new ElementBuffer(mesh->GetFaces());
	mEdges = new ElementBuffer(mesh->GetLines());
	mMesh = mesh;
	mModelMatrix = mat4(1.0f);
}

Body::~Body(void)
{
	if (mVertexes) delete mVertexes;
	if (mFaces) delete mFaces;
	if (mEdges) delete mEdges;
}
