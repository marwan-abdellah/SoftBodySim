#include "engine/solver/SoftBodySolver.h"
#include "common.h"

using namespace glm;

#define DEFAULT_STEPS 10

SoftBodySolver::SoftBodySolver(void)
{
	mWorldParams.gravity = vec3(0, -10.0f, 0);
	mWorldParams.groundLevel = -2.0f;
	mSolverSteps = DEFAULT_STEPS;
}

SoftBodySolver::~SoftBodySolver(void)
{
	mBodies.clear();
}

void SoftBodySolver::SetWorldParameters(SoftBodyWorldParameters &params)
{
	mWorldParams = params;
}

void SoftBodySolver::AddSoftBody(SoftBody *body)
{
	mBodies.push_back(body);
}

void SoftBodySolver::RemoveSoftBody(SoftBody *body)
{
	mBodies.remove(body);
}

void SoftBodySolver::Shutdown(void)
{
	mBodies.clear();
}
