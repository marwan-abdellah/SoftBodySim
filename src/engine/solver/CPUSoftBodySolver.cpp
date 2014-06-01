#include "engine/solver/CPUSoftBodySolver.h"
#include "engine/geometry/Math.h"
#include "common.h"
#include <glm/ext.hpp>

using namespace glm;

static vec3 calculateMassCenter(vec3 *pos, float_t *mass, int n);

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

void CPUSoftBodySolver::SolveShapeMatchConstraint(void)
{
	vec3 mc;
	mat3 A, R;
	FOREACH_R(it, mDescriptors) {
		// calculate mass center after pojection
		mc = calculateMassCenter(&mProjections[it->baseIdx],
								 &mInvMasses[it->baseIdx],
								 it->count);
		// calculate A = sum(mi * (xi - mc) * (x0i - mc0))
		A = mat3();
		REP(i, it->count) {
			vec3 p = mProjections[it->baseIdx + i] - mc;
			A += mInvMasses[it->baseIdx + i] *
				outerProduct(p, mShapes[it->shapeMatching.descriptor].diffs[i]);
		}
		mat3 B = transpose(A) * A;

		// B is symmetrix matrix so it is diagonizable
		vec3 eig = eigenvalues_jacobi(B, 10);

		B = diagonal3x3(eig);

		// calculate squere root of diagonal matrix
		B[0][0] = sqrt(B[0][0]);
		B[1][1] = sqrt(B[1][1]);
		B[2][2] = sqrt(B[2][2]);

		// calculate Rotation matrix
		R = A * inverse(B);

		// calculate target positions and multiply it be constraint stiffness
		// parameter
	    REP(i , it->count) {
			vec3 g = R * mShapes[it->shapeMatching.descriptor].diffs[i] + mc;
			mProjections[it->baseIdx + i] += (g - mProjections[it->baseIdx + i]) * 0.1f;
		}
	}
}

void CPUSoftBodySolver::ProjectSystem(float_t dt)
{
	PredictMotion(dt);

	REP(i, mSolverSteps) {
		SolveShapeMatchConstraint();
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

	// calculate relative locations q0i = x0i - mc0
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
