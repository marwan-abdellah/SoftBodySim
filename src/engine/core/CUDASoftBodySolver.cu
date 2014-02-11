#include "CUDASoftBodySolver.h"

#include "common.h"
#include <cstring>

using namespace std;
using namespace glm;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "CUDASoftBodySolverKernel.h"

struct CUDASoftBodySolver::CollisionBodyInfoDescriptor {
	vec3 *positions;
	CollisionBodyInfo collInfo;
};

struct CUDASoftBodySolver::SoftBodyDescriptor {
	SoftBody *body;
	cudaGraphicsResource *graphics;

	vec3 *positions;
	vec3 *projections;
	vec3 *velocities;
	vec3 *forces;
	float_t *massesInv;
	unsigned int nParticles;

	LinkConstraint *links;
	unsigned int nLinks;

	uint_t *mapping;  /* Mapping between particles positions and vertexes is VertexBuffer.
						 Used for updating Vertex poistions */
	unsigned int nMapping;
};

struct CUDASoftBodySolver::SolverPrivate {
	int			 deviceId;
	cudaDeviceProp  devProp;
	cudaStream_t	stream;

	descriptorArray_t descriptors;
	vector<cudaGraphicsResource*> resArray; /* helper array to map all resources in one call */

	collisionBodyDescriptorArray_t collBodyDescrHost;
	CollisionBodyInfoDescriptor *collBodyDescrDevice;
};

CUDASoftBodySolver::CUDASoftBodySolver(void)
	:
		mCuda(0),
		mInitialized(false),
		mGravity(0, -10.0f, 0)
{
}

CUDASoftBodySolver::~CUDASoftBodySolver(void)
{
	shutdown();
}

bool CUDASoftBodySolver::cudaInitializeDevice(SolverPrivate *cuda)
{
	cudaError_t err;
	cudaDeviceProp  prop;
	memset(&prop, 0x0, sizeof(prop));
	prop.major = 3;
	prop.minor = 5;

	// choose device for us. Prefer with compute capabilities ~ 3.5
	err = cudaChooseDevice(&cuda->deviceId, &prop);
	if (err != cudaSuccess) return false;

	err = cudaSetDevice(cuda->deviceId);
	if (err != cudaSuccess) return false;

	err = cudaGetDeviceProperties(&cuda->devProp, cuda->deviceId);
	if (err != cudaSuccess) return false;
	
	err = cudaStreamCreate(&cuda->stream);
	if (err != cudaSuccess) return false;

	DBG("Choosen CUDA Device: %s", cuda->devProp.name);
	DBG("Multiprocessor count: %d", cuda->devProp.multiProcessorCount);
	DBG("Compute capability: %d.%d", cuda->devProp.major, cuda->devProp.minor);

	return true;
}

bool CUDASoftBodySolver::cudaShutdownDevice(SolverPrivate *cuda)
{
	cudaError_t err;

	err = cudaDeviceSynchronize();
	if (err != cudaSuccess) return false;

	err = cudaDeviceReset();
	if (err != cudaSuccess) return false;

	return true;
}

static void *allocateCUDABuffer(size_t bytes, bool zeroed=false)
{
	cudaError_t err;
	void *ret = NULL;
	err = cudaMalloc(&ret, bytes);
	if (err != cudaSuccess) {
		ERR("%s", cudaGetErrorString(err));
		return NULL;
	}
	if (zeroed) {
		err = cudaMemset(ret, 0x0, bytes);
		if (err != cudaSuccess) {
			ERR("%s", cudaGetErrorString(err));
			return NULL;
		}
	}
	return ret;
}

long CUDASoftBodySolver::cudaAllocateDeviceBuffers(SoftBodyDescriptor *descr)
{
	int bytesArray = 0, bytesMass = 0, bytesMapping = 0, bytesLinks = 0;

	bytesArray = descr->nParticles * sizeof(vec3);
	bytesMass = descr->nParticles * sizeof(float_t);
	bytesMapping = descr->nMapping * sizeof(uint_t);
	bytesLinks = descr->nLinks * sizeof(LinkConstraint);

	descr->positions = (vec3*)allocateCUDABuffer(bytesArray);
	if (!descr->positions) goto on_fail;

	descr->projections = (vec3*)allocateCUDABuffer(bytesArray);
	if (!descr->projections) goto on_fail;

	descr->velocities = (vec3*)allocateCUDABuffer(bytesArray, true);
	if (!descr->velocities) goto on_fail;

	descr->forces  = (vec3*)allocateCUDABuffer(bytesArray, true);
	if (!descr->forces) goto on_fail;

	descr->massesInv = (float_t*)allocateCUDABuffer(bytesMass);
	if (!descr->massesInv) goto on_fail;

	descr->mapping = (uint*)allocateCUDABuffer(bytesMapping);
	if (!descr->mapping) goto on_fail;

	descr->links = (LinkConstraint*)allocateCUDABuffer(bytesLinks);
	if (!descr->links) goto on_fail;

	return 4 * bytesArray + bytesMass + bytesMapping + bytesLinks;

on_fail:
	cudaDeallocateDeviceBuffers(descr);
	return -1;
}

void CUDASoftBodySolver::cudaDeallocateDeviceBuffers(SoftBodyDescriptor *descr)
{
	if (descr->positions) cudaFree(descr->positions);
	if (descr->projections) cudaFree(descr->projections);
	if (descr->velocities) cudaFree(descr->velocities);
	if (descr->forces) cudaFree(descr->forces);
	if (descr->massesInv) cudaFree(descr->massesInv);
	if (descr->links) cudaFree(descr->links);
	if (descr->mapping) cudaFree(descr->mapping);
}

CUDASoftBodySolver::SoftBodyDescriptor CUDASoftBodySolver::cudaCreateDescriptor(SoftBody *body)
{
	SoftBodyDescriptor descr;

	descr.body = body;
	descr.graphics = NULL;
	descr.nParticles = body->mParticles.size();
	descr.nLinks = body->mLinks.size();
	descr.nMapping = body->mMeshVertexParticleMapping.size();

	return descr;
}

bool CUDASoftBodySolver::cudaCopyBodyToDeviceBuffers(SoftBodyDescriptor *descr)
{
	cudaError_t err;

	SoftBody *body = descr->body;

	unsigned int bytesPart = descr->nParticles * sizeof(vec3);
	unsigned int bytesLnk = descr->nLinks * sizeof(LinkConstraint);
	unsigned int bytesMass = descr->nParticles * sizeof(float_t);
	unsigned int bytesMap = descr->nParticles * sizeof(uint_t);

	err = cudaMemcpy(descr->positions, &(body->mParticles[0]), bytesPart, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	err = cudaMemcpy(descr->forces, &(body->mForces[0]), bytesPart, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	err = cudaMemcpy(descr->mapping, &(body->mMeshVertexParticleMapping[0]), bytesMap, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	err = cudaMemcpy(descr->massesInv, &(body->mMassInv[0]), bytesMass, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	err = cudaMemcpy(descr->links, &(body->mLinks[0]), bytesLnk, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	return true;
}

cudaGraphicsResource *cudaRegisterGLGraphicsResource(const GLVertexBuffer *vb)
{
	cudaError_t err;
	cudaGraphicsResource *ret = NULL;
	GLuint id = vb->getVBO(GLVertexBuffer::VERTEX_ATTR_POSITION);
	err = cudaGraphicsGLRegisterBuffer(&ret, id, cudaGraphicsRegisterFlagsNone);
	if (err != cudaSuccess) {
		ERR("Unable to register GL buffer object %d", id);
		return false;
	}
	return ret;
}

bool CUDASoftBodySolver::cudaRegisterVertexBuffers(SoftBodyDescriptor *descr)
{
	const Mesh_t *mesh;

	if (!descr->body) {
		ERR("No SoftBody reference in descriptor!");
		return false;
	}

	mesh = descr->body->getMesh();
	if (!mesh) {
		ERR("No mesh data");
		return false;
	}

	const VertexBuffer *buf = mesh->vertexes;
	if (buf) {
		switch (buf->getType()) {
			case VertexBuffer::OPENGL_BUFFER:
				descr->graphics = cudaRegisterGLGraphicsResource(static_cast<const GLVertexBuffer*>(buf));
				if (!descr->graphics)
					return false;
				break;
			default:
				break;
		}
	}

	return true;
}

bool CUDASoftBodySolver::cudaInitCollisionDescriptors(SolverPrivate *cuda)
{
	cudaError_t err;
	size_t bytes;

	if (!cuda->collBodyDescrHost.size()) return true;

	bytes = cuda->collBodyDescrHost.size() * sizeof(CollisionBodyInfoDescriptor);
	cuda->collBodyDescrDevice = (CollisionBodyInfoDescriptor*)allocateCUDABuffer(bytes);
	if (!cuda->collBodyDescrDevice) return false;

	err = cudaMemcpy(cuda->collBodyDescrDevice, &cuda->collBodyDescrHost[0], bytes, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) {
		cudaFree(cuda->collBodyDescrDevice);
		cuda->collBodyDescrDevice = NULL;
		return false;
	}
	return true;
}

void CUDASoftBodySolver::cudaAppendCollsionDescriptors(collisionBodyDescriptorArray_t *arr, SoftBodyDescriptor *descr)
{
	FOREACH(it, &descr->body->mCollisionBodies) {
		CollisionBodyInfoDescriptor cb;
		cb.positions = descr->positions;
		cb.collInfo = *it;

		arr->push_back(cb);
	}
}

CUDASoftBodySolver::SolverPrivate *CUDASoftBodySolver::cudaContextCreate(softbodyArray_t *bodies)
{
	SolverPrivate *cuda;
	long total_alloc = 0;
	bool res;

	cuda = new SolverPrivate;
	memset(cuda, 0x0, sizeof(SolverPrivate));

	if (!cudaInitializeDevice(cuda)) {
		ERR("CUDA Device initialization failed!");
		delete cuda;
		return NULL;
	}

	FOREACH(it, bodies) {
		if (!*it) continue;

		SoftBodyDescriptor descr = cudaCreateDescriptor(*it);

		long mem = cudaAllocateDeviceBuffers(&descr);
		if (mem == -1) {
			ERR("Unable to allocate memory for SoftBody");
			cudaContextShutdown(cuda);
			return NULL;
		}
		res = cudaCopyBodyToDeviceBuffers(&descr);
		if (!res) {
			ERR("Error occured while copying Soft bodies data to device!");
			ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
			cudaContextShutdown(cuda);
			return NULL;
		}
		res = cudaRegisterVertexBuffers(&descr);
		if (!res) {
			ERR("Error occured registering SoftBody vertex buffers.");
			ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
			cudaContextShutdown(cuda);
			return NULL;
		}
		cuda->descriptors.push_back(descr);
		cudaAppendCollsionDescriptors(&cuda->collBodyDescrHost, &descr);
		cuda->resArray.push_back(descr.graphics);

		total_alloc += mem;
	}
	if (!cudaInitCollisionDescriptors(cuda)) {
		ERR("Error on allocatin collision object data.");
		cudaContextShutdown(cuda);
		return NULL;
	}
	DBG("Allocated %ld bytes on device", total_alloc);

	return cuda;
}

void CUDASoftBodySolver::cudaContextShutdown(SolverPrivate *cuda)
{
	FOREACH(it, &cuda->descriptors)
		cudaDeallocateDeviceBuffers(&(*it));
	if (cuda->collBodyDescrDevice) cudaFree(cuda->collBodyDescrDevice);
	cudaShutdownDevice(cuda);
	delete cuda;
}

bool CUDASoftBodySolver::initialize(softbodyArray_t *bodies)
{
	SolverPrivate *cuda;

	if (mInitialized) return true;

	cuda = cudaContextCreate(bodies);
		if (!cuda) {
		ERR("Unable to create CUDA context.");
		return false;
	}

	mInitialized = true;
	mCuda = cuda;
	return true;
}

void CUDASoftBodySolver::shutdown(void)
{
	if (!mInitialized) return;

	if (mCuda) {
		cudaContextShutdown(mCuda);
		mCuda = NULL;
	}
	mInitialized = false;
}

void CUDASoftBodySolver::updateVertexBuffers(SolverPrivate *cuda, bool async)
{
	cudaError_t err;
	vec3 *ptr;
	int threadCount = 128;

	// map all in one call
	err = cudaGraphicsMapResources(cuda->resArray.size(), &cuda->resArray[0]);
	if (err != cudaSuccess) return;

	FOREACH(it, &cuda->descriptors) {
		size_t size;
		err = cudaGraphicsResourceGetMappedPointer((void**)&ptr, &size, it->graphics);
		if (err != cudaSuccess) {
			ERR("Unable to map VBO pointer");
			return;
		}
		if (size != it->nParticles * sizeof(vec3)) {
			ERR("Invalid size!");
			return;
		}
		int blockCount = it->nParticles / threadCount + 1;
		cudaUpdateVertexBufferKernel<<<blockCount, threadCount >>>(ptr,
				it->positions, it->mapping, it->nParticles);
	}

	cudaGraphicsUnmapResources(cuda->resArray.size(), &cuda->resArray[0]);
}

void CUDASoftBodySolver::updateVertexBuffersAsync(void)
{
	if (mInitialized)
		updateVertexBuffers(mCuda, false); // currently only synch updates
}

void CUDASoftBodySolver::updateVertexBuffers(void)
{
	if (mInitialized)
		updateVertexBuffers(mCuda, false);
}

void CUDASoftBodySolver::projectSystem(SolverPrivate *cuda, float_t dt)
{
	int threadCount = 128;
	FOREACH(it, &cuda->descriptors) {
		int blockCount = it->nParticles / threadCount + 1;

		cudaUpdateVelocitiesKernel<<<blockCount, threadCount>>>(mGravity, it->positions,
				it->projections, it->velocities, it->forces, it->massesInv, dt,
				it->nParticles);

		integrateMotionKernel<<<blockCount, threadCount>>>(dt, it->positions, it->projections,
				it->velocities, it->nParticles);
	}
}

void CUDASoftBodySolver::projectSystem(float_t dt)
{
	if (mInitialized)
		projectSystem(mCuda, dt);
}
