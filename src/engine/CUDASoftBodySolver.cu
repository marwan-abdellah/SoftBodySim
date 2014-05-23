#include "CUDASoftBodySolver.h"

#include "common.h"
#include <cstring>

using namespace std;
using namespace glm;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "CUDASoftBodySolverKernel.h"

#define DEFAULT_SOLVER_STEPS 10

struct CUDASoftBodySolver::SoftBodyDescriptor {
	SoftBody             *body;
	cudaGraphicsResource *graphics;

	vec3                 *positions;
	vec3                 *projections;
	vec3                 *velocities;
	vec3                 *forces;
	float_t              *massesInv;
	unsigned int         nParticles;
	LinkConstraint       *links;
	unsigned int         nLinks;
	uint_t               *mapping;  /* Mapping between particles positions and vertexes 
									   is VertexBuffer.
									 Used for updating Vertex poistions */
	unsigned int         nMapping;
};

struct CUDASoftBodySolver::SolverPrivate {
	int                                deviceId;
	cudaDeviceProp                     devProp;
	cudaStream_t	                   stream;
	int                                solverSteps;

	descriptorArray_t                  descriptors;
	vector<cudaGraphicsResource*>      resArray; /* helper array to map all resources 
												  in one call */

	CollisionPointTriangleConstraint2  *collisions;
	unsigned int                       nCollisions;
};

CUDASoftBodySolver::CUDASoftBodySolver(void)
	:
		mCuda(0),
		mInitialized(false)
{
	mWorldParams.gravity = vec3(0, -10.0f, 0);
	mWorldParams.groundLevel = -2.0f;
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
	if (err != cudaSuccess) goto on_error;

	err = cudaSetDevice(cuda->deviceId);
	if (err != cudaSuccess) goto on_error;

	err = cudaGetDeviceProperties(&cuda->devProp, cuda->deviceId);
	if (err != cudaSuccess) goto on_error;
	
	err = cudaStreamCreate(&cuda->stream);
	if (err != cudaSuccess) goto on_error;

	DBG("Choosen CUDA Device: %s", cuda->devProp.name);

	return true;

on_error:
	ERR("Device initialization error: %s", cudaGetErrorString(cudaGetLastError()));
    return false;
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

	bytesArray   = descr->nParticles * sizeof(vec3);
	bytesMass    = descr->nParticles * sizeof(float_t);
	bytesMapping = descr->nMapping * sizeof(uint_t);
	bytesLinks   = descr->nLinks * sizeof(LinkConstraint);

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
	if (descr->positions)   cudaFree(descr->positions);
	if (descr->projections) cudaFree(descr->projections);
	if (descr->velocities)  cudaFree(descr->velocities);
	if (descr->forces)      cudaFree(descr->forces);
	if (descr->massesInv)   cudaFree(descr->massesInv);
	if (descr->links)       cudaFree(descr->links);
	if (descr->mapping)     cudaFree(descr->mapping);
}

CUDASoftBodySolver::SoftBodyDescriptor CUDASoftBodySolver::cudaCreateDescriptor(SoftBody *body)
{
	SoftBodyDescriptor descr;

	descr.body       = body;
	descr.graphics   = NULL;
	descr.nParticles = body->mParticles.size();
	descr.nLinks     = body->mLinks.size();
	descr.nMapping   = body->mMeshVertexParticleMapping.size();

	return descr;
}

bool CUDASoftBodySolver::cudaCopyBodyToDeviceBuffers(SoftBodyDescriptor *descr)
{
	cudaError_t err;

	SoftBody *body = descr->body;

	unsigned int bytesPart = descr->nParticles * sizeof(vec3);
	unsigned int bytesLnk  = descr->nLinks * sizeof(LinkConstraint);
	unsigned int bytesMass = descr->nParticles * sizeof(float_t);
	unsigned int bytesMap  = descr->nMapping * sizeof(uint_t);

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

cudaGraphicsResource *cudaRegisterGLGraphicsResource(const VertexBuffer *vb)
{
	cudaError_t err;
	cudaGraphicsResource *ret = NULL;
	GLuint id = vb->GetVBO();
	err = cudaGraphicsGLRegisterBuffer(&ret, id, cudaGraphicsRegisterFlagsNone);
	if (err != cudaSuccess) {
		ERR("Unable to register GL buffer object %d", id);
		return false;
	}
	return ret;
}

bool CUDASoftBodySolver::cudaRegisterVertexBuffers(SoftBodyDescriptor *descr)
{
	if (!descr->body) {
		ERR("No SoftBody reference in descriptor!");
		return false;
	}

	const VertexBuffer *buf = descr->body->GetVertexes();
	if (buf)
		descr->graphics = cudaRegisterGLGraphicsResource(buf);

	return true;
}

void CUDASoftBodySolver::cudaUpdateConstraintStiffness(SoftBodyDescriptor
		*descr, int solverSteps)
{
	int blocks = descr->nLinks / 128;
	calculateLinkStiffness<<<blocks, 128>>>(solverSteps, descr->links, descr->nLinks);
}

CUDASoftBodySolver::SolverPrivate *CUDASoftBodySolver::cudaContextCreate(softbodyList_t *bodies)
{
	SolverPrivate *cuda;
	long total_alloc = 0;
	bool res;

	cuda = new SolverPrivate;
	memset(cuda, 0x0, sizeof(SolverPrivate));

	cuda->solverSteps = DEFAULT_SOLVER_STEPS;

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
		cudaUpdateConstraintStiffness(&descr, cuda->solverSteps);

		cuda->descriptors.push_back(descr);
		cuda->resArray.push_back(descr.graphics);

		total_alloc += mem;
	}
	DBG("Allocated %ld bytes on device", total_alloc);

	return cuda;
}

void CUDASoftBodySolver::cudaContextShutdown(SolverPrivate *cuda)
{
	FOREACH(it, &cuda->descriptors)
		cudaDeallocateDeviceBuffers(&(*it));
	cudaShutdownDevice(cuda);
	delete cuda;
}

bool CUDASoftBodySolver::initialize(void)
{
	SolverPrivate *cuda;

	if (mInitialized) return true;

	cuda = cudaContextCreate(&mBodies);
		if (!cuda) {
		ERR("Unable to create CUDA context.");
		return false;
	}

	mBodies.clear();
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
	MeshData::Vertex *ptr;
	int threadsPerBlock = 128;

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
		int blockCount = it->nMapping / threadsPerBlock + 1;
		cudaUpdateVertexBufferKernel<<<blockCount, threadsPerBlock >>>(
				ptr, it->positions, it->mapping, it->nMapping);
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
	int threadsPerBlock = 128;
	FOREACH(it, &cuda->descriptors) {
		int blockCount = it->nParticles / threadsPerBlock + 1;

		cudaUpdateVelocitiesKernel<<<blockCount,
			threadsPerBlock>>>(mWorldParams.gravity, it->positions,
				it->projections, it->velocities, it->forces, it->massesInv, dt,
				it->nParticles);

		threadsPerBlock = MAX_LINKS;
		blockCount = it->nLinks / threadsPerBlock + 1;

		for (int i = 0; i < cuda->solverSteps; i++)
			solveConstraints<<<blockCount, threadsPerBlock>>>(1, it->links,
					it->projections, it->massesInv, it->nLinks);

		threadsPerBlock = 128;
		blockCount = it->nParticles / threadsPerBlock + 1;
		integrateMotionKernel<<<blockCount, threadsPerBlock>>>(dt, it->positions, it->projections,
				it->velocities, it->nParticles);
//		ERR("Cuda err: %s", cudaGetErrorString(cudaGetLastError()));
	}
}

void CUDASoftBodySolver::projectSystem(float_t dt)
{
	if (mInitialized)
		projectSystem(mCuda, dt);
}

void CUDASoftBodySolver::setWorldParameters(SoftBodyWorldParameters &params)
{
	mWorldParams = params;
}

void CUDASoftBodySolver::addSoftBodies(softbodyList_t &bodies)
{
	FOREACH_R(it, bodies)
		mBodies.push_back(*it);
}

void CUDASoftBodySolver::removeBodies(softbodyList_t *bodies)
{
}

void CUDASoftBodySolver::addSoftBody(SoftBody *body)
{
	mBodies.push_back(body);
}

void CUDASoftBodySolver::removeSoftBody(SoftBody *body)
{
	mBodies.remove(body);
}
