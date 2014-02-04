#include "CUDASoftBodySolver.h"

using namespace std;
using namespace glm;

CUDASoftBodySolver::CUDASoftBodySolver(void)
	:
		mInitialized(false)
{
}

CUDASoftBodySolver::~CUDASoftBodySolver(void)
{
	if (mInitialized)
		terminate();
}

bool CUDASoftBodySolver::initializeDevice(void)
{
	cudaError_t error;
	int count;

	error = cudaGetDeviceCount(&count);
	if (error != cudaSuccess)
		return false;

	// take by default last device
	mDevId = count - 1;

	error = cudaSetDevice(mDevId);
	if (error != cudaSuccess)
		return false;
	
	error = cudaStreamCreate(&mStream);
	if (error != cudaSuccess)
		return false;

	return true;
}

bool CUDASoftBodySolver::copyBodiesToDevice(vector<SoftBody> &bodies)
{
	int cells2alloc = 0;
	int idx = 0, idx2= 0;
	cudaError_t error;

	FOREACH(it, bodies)
		cells2alloc += it->mParticles.size();

	for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; ++type) {
		error = cudaMalloc(&mArray[type], cells2alloc * sizeof(glm::vec3));
		if (error != cudaSuccess)
			return false;
	}

	error = cudaMalloc(&mMassInv, cells2alloc);
	if (error != cudaSuccess)
		return false;

	cudaMemset(mArray[ARRAY_FORCES], 0x0, cells2alloc);

	cells2alloc = 0;
	FOREACH(it, bodies)
		cells2alloc += it->mLinks.size();

	error = cudaMalloc(&mLinks, cells2alloc * sizeof(glm::uvec2));
	if (error != cudaSuccess) {
		return false;
	}
	cudaMalloc(&mLinksRestLength2, cells2alloc * sizeof(glm::float_t));
	if (error != cudaSuccess) {
		return false;
	}

	FOREACH(it, bodies) {
		SoftBodyDescriptor descr;
		descr.body = &(*it);
		descr.vertexBaseIdx = idx;
		descr.linksBaseIdx = idx2;

		idx += it->mParticles.size();
		idx2 += it->mLinks.size();

		unsigned int bytes1 = it->mParticles.size() * sizeof(glm::vec3);
		unsigned int bytes2 = it->mLinks.size() * sizeof(glm::uvec2);
		unsigned int bytes3 = it->mLinks.size() * sizeof(glm::float_t);

		unsigned int offset = idx * sizeof(glm::vec3);
		unsigned int offset2 = idx2 * sizeof(glm::uvec2);
		unsigned int offset3 = idx2 * sizeof(glm::float_t);

		cudaMemcpy(mArray[ARRAY_POSITIONS] + offset, &(it->mParticles[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(mArray[ARRAY_PROJECTIONS] + offset, &(it->mParticles[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(mArray[ARRAY_VELOCITIES] + offset, &(it->mVelocities[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(mArray[ARRAY_FORCES] + offset, &(it->mForces[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemset(mMassInv + offset3, it->mMassInv, bytes1);

		vector<uvec2> tmp(it->mLinks.size());
		vector<float_t> tmp2(it->mLinks.size());

		FOREACH(lnk, it->mLinks)
		{
			tmp.push_back(lnk->index);
			tmp2.push_back(lnk->restLength);
		}

		cudaMemcpy(mLinks + offset2, &tmp[0], bytes2, cudaMemcpyHostToDevice);
		cudaMemcpy(mLinksRestLength2 + offset3, &tmp2[0], bytes3, cudaMemcpyHostToDevice);

		mDescriptors.push_back(descr);
	}

	return true;
}

void CUDASoftBodySolver::freeBodies(void)
{
	for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; type++) {
		if (mArray[type]) cudaFree(mArray[type]);
		mArray[type] = NULL;
	}

	if (mLinks) cudaFree(mLinks);
	mLinks = NULL;
	if (mLinksRestLength2) cudaFree(mLinksRestLength2);
	mLinksRestLength2 = NULL;
	mDescriptors.clear();
}

void CUDASoftBodySolver::initialize(vector<SoftBody> &bodies)
{
	if (!initializeDevice()) {
		ERR("CUDA Device initialization failed!");
		shutdownDevice();
		return;
	}

	if (!copyBodiesToDevice(bodies)) {
		ERR("Unable to copy Soft bodies to device!");
		freeBodies();
		return;
	}

	mInitialized = true;
}

void CUDASoftBodySolver::shutdownDevice(void)
{
	cudaStreamDestroy(mStream);
}

void CUDASoftBodySolver::terminate(void)
{
	shutdownDevice();
}

