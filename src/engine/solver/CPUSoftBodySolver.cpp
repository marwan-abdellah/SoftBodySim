#include "engine/solver/CPUSoftBodySolver.h"
#include "common.h"

using namespace glm;


CPUSoftBodySolver::CPUSoftBodySolver()
{
}

CPUSoftBodySolver::~CPUSoftBodySolver()
{
	Shutdown();
}

void CPUSoftBodySolver::PredictMotion(float dt)
{
	REP(i, mPositions.size()) {
		mVelocities[i] += dt * mInvMasses[i] * (mForces[i] + mWorldParams.gravity);
		mVelocities[i] *= 0.99;
		mProjections[i] = mPositions[i] + mVelocities[i] * dt;
	}
}

void CPUSoftBodySolver::IntegrateSystem(float dt)
{
	REP(i, mPositions.size()) {
		mVelocities[i] = (mProjections[i] - mPositions[i]) / dt;
		mPositions[i] = mProjections[i];
	}
}

void CPUSoftBodySolver::SolveGroundCollisions(void)
{
	float_t ground = mWorldParams.groundLevel;
	REP(i, mPositions.size())
		mProjections[i][1] = mProjections[i][1] < ground ? ground : mProjections[i][1];
}

void CPUSoftBodySolver::ProjectSystem(float_t dt)
{
	PredictMotion(dt);
	REP(i, mSolverSteps) {
		SolveGroundCollisions();
	}

	IntegrateSystem(dt);
}

bool CPUSoftBodySolver::Initialize(void)
{
	FOREACH_R(it, mBodies)
		AddSoftBody(*it);
	return true;
}

void CPUSoftBodySolver::Shutdown(void)
{
	mPositions.clear();
	mProjections.clear(); mForces.clear();
	mVelocities.clear();
	mInvMasses.clear();
	mLinks.clear();
	mMapping.clear();
	mDescriptors.clear();

	SoftBodySolver::Shutdown();
}

static vec3 calculateMassCenter(vec3 *pos, float_t *mass, int n)
{
	double masssum = 0;
	vec3 xmsum = vec3(0,0,0);

	//calculate sum(xi * mi) amd sum(mi)
	REP(i, n) {
		xmsum += pos[i] * mass[i];
		masssum += mass[i];
	}

	return xmsum / (float_t)masssum;
}

void CPUSoftBodySolver::AddShapeDescriptor(SoftBody *obj)
{
	ShapeDescriptor ret;

	ret.mc0 = calculateMassCenter(
			&(obj->mParticles[0]), &(obj->mMassInv[0]), obj->mParticles.size());

	// calculate differences q0i = x0i - mc0
	REP(i, obj->mParticles.size()) {
		vec3 q = obj->mParticles[i] - ret.mc0;
		ret.diffs.push_back(q);
	}
	mShapes.push_back(ret);
}

void CPUSoftBodySolver::UpdateVertexBuffers(void)
{
	FOREACH(it, &mDescriptors) {
		for(int i = 0; i < it->nMapping; i++) {
			int idx = mMapping[it->mappingBaseIdx + i];
			mVertexes[i] = mPositions[it->baseIdx + idx];
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
	descr.shapeMatching.descriptor = mShapes.size();

	AddShapeDescriptor(b);

	mDescriptors.push_back(descr);

	mVertexes.reserve(b->mMeshVertexParticleMapping.size());
	mPositions.insert(mPositions.end(), b->mParticles.begin(), b->mParticles.end());
	mProjections.insert(mProjections.end(), b->mParticles.begin(), b->mParticles.end());
	mVelocities.resize(mVelocities.size() + b->mParticles.size());
	mForces.resize(mPositions.size() + b->mParticles.size());
	mInvMasses.insert(mInvMasses.end(), b->mMassInv.begin(), b->mMassInv.end());

	mLinks.insert(mLinks.end(), b->mLinks.begin(), b->mLinks.end());
	mMapping.insert(mMapping.end(), b->mMeshVertexParticleMapping.begin(),
			b->mMeshVertexParticleMapping.end());

	mVertexes.resize(mVertexes.capacity());
}
