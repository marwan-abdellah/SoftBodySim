#include "engine/solver/SoftBodySolver.h"
#include "common.h"

using namespace glm;

SoftBodySolver::SoftBodySolver(void)
{
	mWorldParams.gravity = vec3(0, -10.0f, 0);
	mWorldParams.groundLevel = -2.0f;
}

SoftBodySolver::~SoftBodySolver(void)
{
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
