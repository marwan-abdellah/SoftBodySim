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

void CPUSoftBodySolver::SolveGroundWallCollisions(void)
{
	float_t ground = mWorldParams.groundLevel;
	float_t left = mWorldParams.leftWall;
	float_t right = mWorldParams.rightWall;
	float_t front = mWorldParams.frontWall;
	float_t back = mWorldParams.backWall;

	REP(i, mPositions.size()) {
		mProjections[i][1] = mProjections[i][1] < ground ? ground : mProjections[i][1];
		mProjections[i][0] = mProjections[i][0] < left ? left : mProjections[i][0];
		mProjections[i][0] = mProjections[i][0] > right ? right : mProjections[i][0];
		mProjections[i][2] = mProjections[i][2] > front ? front : mProjections[i][2];
		mProjections[i][2] = mProjections[i][2] < back ? back : mProjections[i][2];
	}
}

void CPUSoftBodySolver::SolveShapeMatchConstraint(void)
{
	vec3 mc;
	mat3 A, R;
	mat3 E;
	float_t sping;
	FOREACH_R(it, mDescriptors) {
		// calculate mass center after pojection
		mc = calculateMassCenter(&mProjections[it->baseIdx],
								 &mInvMasses[it->baseIdx],
								 it->count);
		// calculate A = sum(mi * (xi - mc) * (x0i - mc0))
		//ERR("Mass center: %f %f %f", mc[0], mc[1], mc[2]);
		A = mat3();
		REP(i, it->count) {
			vec3 p = mProjections[it->baseIdx + i] - mc;
			A += mInvMasses[it->baseIdx + i] *
				outerProduct(mShapes[it->shapeMatching.descriptor].diffs[i], p);
		}
		mat3 B = transpose(A) * A;

		// B is symmetrix matrix so it is diagonizable
		vec3 eig = eigenvalues_jacobi(B, 10, E);

		mat3 D = diagonal3x3(eig);
		// calculate squere root of diagonal matrix
		D[0][0] = sqrt(D[0][0]);
		D[1][1] = sqrt(D[1][1]);
		D[2][2] = sqrt(D[2][2]);
		
		B = inverse(E) * D * E;

		// calculate Rotation matrix
		R = A * inverse(B);

		// calculate target positions and multiply it be constraint stiffness
		sping = it->body->mSpringiness;
	    REP(i , it->count) {
			vec3 g = R * mShapes[it->shapeMatching.descriptor].diffs[i] + mc;
			mProjections[it->baseIdx + i] += (g - mProjections[it->baseIdx + i]) * sping;
		}
	}
}

void CPUSoftBodySolver::ProjectSystem(float_t dt)
{
	if (!mInitialized) return;

	PredictMotion(dt);

	REP(i, mSolverSteps) {
		SolveShapeMatchConstraint();
		SolveGroundWallCollisions();
	}

	IntegrateSystem(dt);
}

bool CPUSoftBodySolver::Initialize(void)
{
	FOREACH_R(it, mBodies)
		AddSoftBody(*it);

	mInitialized = true;
	return mInitialized;
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
	mShapes.clear();

	SoftBodySolver::Shutdown();
	mInitialized = false;
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
	if (!mInitialized) return;

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

	SoftBodySolver::AddSoftBody(b);
}
