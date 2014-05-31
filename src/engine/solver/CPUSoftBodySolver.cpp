#include "engine/solver/CPUSoftBodySolver.h"
#include "common.h"

using namespace glm;


CPUSoftBodySolver::CPUSoftBodySolver()
{
}

CPUSoftBodySolver::~CPUSoftBodySolver()
{
}

void CPUSoftBodySolver::ProjectSystem(float_t dt)
{
	// PredictMotion();
	// for (i = 0; i < solverSteps; i++)
	//    solverConstraints();
	// IntegrateSystem();
}

bool CPUSoftBodySolver::Initialize(void)
{
	FOREACH_R(it, mBodies)
		AddSoftBody(*it);
	mVertexes.resize(mVertexes.capacity());
	return true;
}

void CPUSoftBodySolver::Shutdown(void)
{
	mPositions.clear();
	mProjections.clear();
	mForces.clear();
	mVelocities.clear();
	mInvMasses.clear();
	mLinks.clear();
	mMapping.clear();
	mDescriptors.clear();
}

void CPUSoftBodySolver::UpdateVertexBuffers(void)
{
	FOREACH(it, &mDescriptors) {
		for(int i = 0; i < it->nMapping; i++) {
			int idx = mMapping[it->mappingBaseIdx + i];
			mVertexes[i] = mPositions[idx];
		}
		VertexBuffer *vb = it->body->GetVertexes();
		vb->SetVertexes(&mVertexes[0]);
	}
}

void CPUSoftBodySolver::AddSoftBody(SoftBody *b)
{
	SoftBodyDescriptor descr;

	descr.body = b;
	descr.baseIdx = mPositions.size();
	descr.count = b->mParticles.size();
	descr.linkBaseIdx = mLinks.size();
	descr.linkCount = b->mLinks.size();
	descr.mappingBaseIdx = mMapping.size();
	descr.nMapping = b->mMeshVertexParticleMapping.size();

	mDescriptors.push_back(descr);

	mVertexes.reserve(b->mParticles.size());
	mPositions.insert(mPositions.end(), b->mParticles.begin(), b->mParticles.end());
	mProjections.insert(mPositions.end(), b->mParticles.begin(), b->mParticles.end());
	mVelocities.resize(mVelocities.size() + b->mParticles.size());
	mForces.resize(mPositions.size() + b->mParticles.size());
	mInvMasses.insert(mInvMasses.end(), b->mMassInv.begin(), b->mMassInv.end());

	mLinks.insert(mLinks.end(), b->mLinks.begin(), b->mLinks.end());
	mMapping.insert(mMapping.end(), b->mMeshVertexParticleMapping.begin(),
			b->mMeshVertexParticleMapping.end());
}
