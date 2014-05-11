#include "SoftBody.h"
#include "common.h"
#include <cstring>

using namespace glm;
using namespace std;


SoftBody::SoftBody(float_t mass, float_t springness, float_t damping, 
				   MeshData &mesh) :
	mEdges(0),
	mVertexes(0),
	mFaces(0)
{
	mSpringiness = springness;
	mDamping = damping;
	mParticles = mesh.nodes;
	mMeshVertexParticleMapping = mesh.vertexesNodes;

	mMassInv.resize(mParticles.size());
	mVelocities.resize(mParticles.size());
	mForces.resize(mParticles.size());
	mLinks.resize(mesh.nodesLinks.size());

	mass /= 1.0;
	FOREACH(it, &mMassInv)
		*it = mass;
	mMassInv[mMassInv.size() - 1] = 0.0f;

	for(unsigned int i = 0; i < mesh.nodesLinks.size(); i++) {
		LinkConstraint lnk;
		lnk.index = mesh.nodesLinks[i];
		lnk.restLength = length(mParticles[lnk.index[0]] - mParticles[lnk.index[1]]);
		lnk.stiffness = springness;
		mLinks[i] = lnk;
	}
	// FIXME add volume constraint

	mVertexes = new VertexBuffer(mesh.vertexes);
	mFaces = new ElementBuffer(mesh.faces);
	mEdges = new ElementBuffer(mesh.edges);
}

SoftBody::~SoftBody(void)
{
	if (mVertexes) delete mVertexes;
	if (mFaces) delete mFaces;
	if (mEdges) delete mEdges;
}
