#include "SoftBody.h"
#include "common.h"
#include <cstring>

using namespace glm;
using namespace std;


SoftBody::SoftBody(float_t mass, float_t springness, 
				   MeshData *mesh) :
	Body(mesh)
{
	mSpringiness = springness;
	mParticles = mesh->GetNodes();
	mTriangles = mesh->GetNodesTriangles();
	mMeshVertexParticleMapping = mesh->GetNodesLayout();

	mMassInv.resize(mParticles.size());
	mVelocities.resize(mParticles.size());
	mForces.resize(mParticles.size());
	mLinks.resize(mesh->GetNodesConnections().size());

	mMesh = mesh;

	mass = 1.0 / mass;
	FOREACH(it, &mMassInv)
		*it = mass;

	for(unsigned int i = 0; i < mesh->GetNodesConnections().size(); i++) {
		LinkConstraint lnk;
		lnk.index = mesh->GetNodesConnections()[i];
		lnk.restLength = length(mParticles[lnk.index[0]] - mParticles[lnk.index[1]]);
		lnk.stiffness = springness;
		mLinks[i] = lnk;
	}
	// FIXME add volume constraint
}

