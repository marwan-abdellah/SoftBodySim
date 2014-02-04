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

bool CUDASoftBodySolver::copyBodiesToDevice(softbodyArray_t *bodies)
{
	int cells2alloc = 0;
	int idx = 0, idx2= 0;
	cudaError_t error;

	FOREACH(it, bodies)
		cells2alloc += (*it)->mParticles.size();

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
		cells2alloc += (*it)->mLinks.size();

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
		SoftBody *body = *it;
		descr.body = *it;
		descr.vertexBaseIdx = idx;
		descr.linksBaseIdx = idx2;

		idx += body->mParticles.size();
		idx2 += body->mLinks.size();

		unsigned int bytes1 = body->mParticles.size() * sizeof(glm::vec3);
		unsigned int bytes2 = body->mLinks.size() * sizeof(glm::uvec2);
		unsigned int bytes3 = body->mLinks.size() * sizeof(glm::float_t);

		unsigned int offset = idx * sizeof(glm::vec3);
		unsigned int offset2 = idx2 * sizeof(glm::uvec2);
		unsigned int offset3 = idx2 * sizeof(glm::float_t);

		cudaMemcpy(mArray[ARRAY_POSITIONS] + offset, &(body->mParticles[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(mArray[ARRAY_PROJECTIONS] + offset, &(body->mParticles[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(mArray[ARRAY_VELOCITIES] + offset, &(body->mVelocities[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(mArray[ARRAY_FORCES] + offset, &(body->mForces[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemset(mMassInv + offset3, body->mMassInv, bytes1);

		vector<uvec2> tmp(body->mLinks.size());
		vector<float_t> tmp2(body->mLinks.size());

		FOREACH_R(lnk, body->mLinks)
		{
			tmp.push_back(lnk->index);
			tmp2.push_back(lnk->restLength);
		}

		cudaMemcpy(mLinks + offset2, &tmp[0], bytes2, cudaMemcpyHostToDevice);
		cudaMemcpy(mLinksRestLength2 + offset3, &tmp2[0], bytes3, cudaMemcpyHostToDevice);

		mDescriptorMap.insert(make_pair(body, descr));
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
	//mDescriptorMap.clear();
}

void CUDASoftBodySolver::initialize(softbodyArray_t *bodies)
{
	if (mInitialized) return;

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

void CUDASoftBodySolver::terminate(void)
{
	cudaStreamDestroy(mStream);
	mInitialized = false;
}


void CUDASoftBodySolver::copySBDataToGLVertexBuffer(descriptorArray_t *desc, vertexBufferArray_t *vbs)
{
	cudaError_t err;
	vector<cudaGraphicsResource*> resources;
	cudaGraphicsResource *res;
	FOREACH(it, vbs) {
		GLVertexBuffer *glvb = (GLVertexBuffer*)*it;
		err = cudaGraphicsGLRegisterBuffer(&res, glvb->getVBO(GLVertexBuffer::VERTEX_ATTR_POSITION),
				cudaGraphicsRegisterFlagsNone);
		if (err != cudaSuccess)
			ERR("Registering GL buffer failed.");
		resources.push_back(res);
	}
	err = cudaGraphicsMapResources(resources.size(), &resources[0], mStream);
	if (err != cudaSuccess) {
		ERR("Failed to map device resources");
		return;
	}

	FOREACH(it, &resources) {
		void *ptr;
		size_t size;
		err = cudaGraphicsResourceGetMappedPointer(&ptr, &size, *it);
		if (err != cudaSuccess)
			return;
	}
}

void CUDASoftBodySolver::copySBDataToCPUVertexBuffer(descriptorArray_t *desc, vertexBufferArray_t *vb)
{
	// FIXME
	// implement later
}

void CUDASoftBodySolver::copySBDataToVertexBuffers(softbodyArray_t *bodies, vertexBufferArray_t *vbs)
{
	descriptorArray_t glBuffs;
	descriptorArray_t cpuBuffs;
	int idx = 0;

	FOREACH(body, bodies) {
		descriptorMap_t::iterator it = mDescriptorMap.find(*body);
		if (it == mDescriptorMap.end()) {
			ERR("SoftBody object not managed by solver!");
			idx++;
			continue;
		}
		SoftBodyDescriptor *d = &(it->second);
		VertexBuffer *vb = vbs->at(idx);
		switch(vb->getType()) {
			case VertexBuffer::OPENGL_BUFFER:
				glBuffs.push_back(d);
				break;
			case VertexBuffer::CPU_BUFFER:
				cpuBuffs.push_back(d);
				break;
		}
		idx++;
	}
	copySBDataToGLVertexBuffer(&glBuffs, vbs);
	copySBDataToCPUVertexBuffer(&glBuffs, vbs);
}
