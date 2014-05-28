#include "CUDASoftBodySolver.h"

#include "common.h"
#include <cstring>

using namespace std;
using namespace glm;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "CUDASoftBodySolverKernel.h"

#define DEFAULT_SOLVER_STEPS 10
#define DEFAULT_CELL_SIZE 1.0

class CUDAContext {
public:
	typedef std::vector<SoftBodyDescriptor> descriptorArray_t;

	//cudaContextCreate(softbodyList_t*);
	CUDAContext(softbodyList_t *list);

	//void cudaContextShutdown(SolverPrivate*);
	~CUDAContext(void);

	bool InitDevice();
	bool ShutdownDevice();

	SoftBodyDescriptor CreateDescriptor(SoftBody *body);
	long AllocateDeviceBuffers(SoftBodyDescriptor*);
	void DeallocateDeviceBuffers(SoftBodyDescriptor*);
	bool CopyBodyToDeviceBuffers(SoftBodyDescriptor*);
	bool RegisterVertexBuffers(SoftBodyDescriptor *descr);
	void UpdateConstraintStiffness(SoftBodyDescriptor *descr, int mSolverSteps);
	void UpdateVertexBuffers(bool async);
	void ProjectSystem(float_t dt, CUDASoftBodySolver::SoftBodyWorldParameters
			&parms);
	cudaGraphicsResource *RegisterGLGraphicsResource(const VertexBuffer *vb);
	bool InitCellIDS();
	bool InitBodyDescriptors();
private:
	int                                mDeviceId;
	cudaDeviceProp                     mDevProp;
	cudaStream_t                       mStream;
	int                                mSolverSteps;

	descriptorArray_t                  mDescriptors;
	SoftBodyDescriptor				   *mDescriptorsDev;

	vector<cudaGraphicsResource*>      mResArray; /* helper array to map all resources 
												  in one call */

	// collision detection using spatial hashing
	struct {
		CellID                          *devPtr;
		unsigned int                     count;
	} mCellIDS;
};

bool CUDAContext::InitDevice()
{
	cudaError_t err;
	cudaDeviceProp  prop;
	memset(&prop, 0x0, sizeof(prop));
	prop.major = 3;
	prop.minor = 5;

	// choose device for us. Prefer with compute capabilities ~ 3.5
	err = cudaChooseDevice(&mDeviceId, &prop);
	if (err != cudaSuccess) goto on_error;

	err = cudaSetDevice(mDeviceId);
	if (err != cudaSuccess) goto on_error;

	err = cudaGetDeviceProperties(&mDevProp, mDeviceId);
	if (err != cudaSuccess) goto on_error;
	
	err = cudaStreamCreate(&mStream);
	if (err != cudaSuccess) goto on_error;

	DBG("Choosen CUDA Device: %s", mDevProp.name);

	return true;

on_error:
	ERR("Device initialization error: %s", cudaGetErrorString(cudaGetLastError()));
	return false;
}

bool CUDAContext::ShutdownDevice()
{
	cudaError_t err;

	FOREACH(it, &mDescriptors)
		DeallocateDeviceBuffers(&(*it));

	if (mCellIDS.devPtr)
		cudaFree(mCellIDS.devPtr);

	if (mDescriptorsDev)
		cudaFree(mDescriptorsDev);

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

long CUDAContext::AllocateDeviceBuffers(SoftBodyDescriptor *descr)
{
	int bytesArray, bytesMass, bytesMapping, bytesLinks, bytesTriangles;

	bytesArray   = descr->nParticles * sizeof(vec3);
	bytesMass    = descr->nParticles * sizeof(float_t);
	bytesMapping = descr->nMapping * sizeof(uint_t);
	bytesLinks   = descr->nLinks * sizeof(LinkConstraint);
	bytesTriangles = descr->nTriangles * sizeof(uvec3);

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

	descr->triangles = (uvec3*)allocateCUDABuffer(bytesTriangles);
	if (!descr->triangles) goto on_fail;

	return 4 * bytesArray + bytesMass + bytesMapping + bytesLinks;

on_fail:
	DeallocateDeviceBuffers(descr);
	return -1;
}

void CUDAContext::DeallocateDeviceBuffers(SoftBodyDescriptor *descr)
{
	if (descr->positions)   cudaFree(descr->positions);
	if (descr->projections) cudaFree(descr->projections);
	if (descr->velocities)  cudaFree(descr->velocities);
	if (descr->forces)      cudaFree(descr->forces);
	if (descr->massesInv)   cudaFree(descr->massesInv);
	if (descr->links)       cudaFree(descr->links);
	if (descr->mapping)     cudaFree(descr->mapping);
	if (descr->triangles)   cudaFree(descr->triangles);
}

SoftBodyDescriptor CUDAContext::CreateDescriptor(SoftBody *body)
{
	SoftBodyDescriptor descr;

	descr.body       = body;
	descr.graphics   = NULL;
	descr.nParticles = body->mParticles.size();
	descr.nLinks     = body->mLinks.size();
	descr.nMapping   = body->mMeshVertexParticleMapping.size();
	descr.nTriangles = body->mTriangles.size();
	descr.baseIdx    = mCellIDS.count;

	return descr;
}

bool CUDAContext::CopyBodyToDeviceBuffers(SoftBodyDescriptor *descr) {
	cudaError_t err;

	SoftBody *body = descr->body;

	unsigned int bytesPart = descr->nParticles * sizeof(vec3);
	unsigned int bytesLnk  = descr->nLinks * sizeof(LinkConstraint);
	unsigned int bytesMass = descr->nParticles * sizeof(float_t);
	unsigned int bytesMap  = descr->nMapping * sizeof(uint_t);
	unsigned int bytesTria = descr->nTriangles * sizeof(uvec3);

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

	err = cudaMemcpy(descr->triangles, &(body->mTriangles[0]), bytesTria, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	return true;
}

cudaGraphicsResource *CUDAContext::RegisterGLGraphicsResource(const VertexBuffer *vb)
{
	cudaError_t err;
	cudaGraphicsResource *ret = NULL;
	GLuint id = vb->GetVBO();
	err = cudaGraphicsGLRegisterBuffer(&ret, id, cudaGraphicsRegisterFlagsNone);
	if (err != cudaSuccess) {
		ERR("Unable to register GL buffer object %d", id);
		return NULL;
	}
	return ret;
}

bool CUDAContext::RegisterVertexBuffers(SoftBodyDescriptor *descr)
{
	if (!descr->body) {
		ERR("No SoftBody reference in descriptor!");
		return false;
	}

	const VertexBuffer *buf = descr->body->GetVertexes();
	if (buf)
		descr->graphics = RegisterGLGraphicsResource(buf);

	return true;
}

void CUDAContext::UpdateConstraintStiffness(SoftBodyDescriptor
		*descr, int mSolverSteps)
{
	int blocks = descr->nLinks / 128;
	calculateLinkStiffness<<<blocks, 128>>>(mSolverSteps, descr->links, descr->nLinks);
}

bool CUDAContext::InitBodyDescriptors()
{
	cudaError_t err;
	size_t bytes = sizeof(SoftBodyDescriptor) * mDescriptors.size();

	mDescriptorsDev = (SoftBodyDescriptor*)allocateCUDABuffer(bytes);
	if (!mDescriptorsDev) return false;

	err = cudaMemcpy(mDescriptorsDev, &mDescriptorsDev[0], bytes, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	return true;
}

bool CUDAContext::InitCellIDS()
{
	if (!mCellIDS.count) {
		ERR("Invalid cell ids count!");
		return false;
	}

	mCellIDS.devPtr = (CellID*)allocateCUDABuffer(sizeof(CellID) * mCellIDS.count);
	if (!mCellIDS.devPtr)
		return false;

	return true;
}

CUDAContext::CUDAContext(softbodyList_t *bodies)
{
	long total_alloc = 0;
	bool res;

	mCellIDS.count = 0;
	mSolverSteps = DEFAULT_SOLVER_STEPS;

	if (!InitDevice()) {
		ERR("CUDA Device initialization failed!");
		return;
	}

	FOREACH(it, bodies) {
		if (!*it) continue;

		SoftBodyDescriptor descr = CreateDescriptor(*it);

		long mem = AllocateDeviceBuffers(&descr);
		if (mem == -1) {
			ERR("Unable to allocate memory for SoftBody");
			ShutdownDevice();
			return;
		}
		res = CopyBodyToDeviceBuffers(&descr);
		if (!res) {
			ERR("Error occured while copying Soft bodies data to device!");
			ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
			ShutdownDevice();
			return;
		}
		res = RegisterVertexBuffers(&descr);
		if (!res) {
			ERR("Error occured registering SoftBody vertex buffers.");
			ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
			ShutdownDevice();
			return;
		}
		UpdateConstraintStiffness(&descr, mSolverSteps);

		mDescriptors.push_back(descr);
		mResArray.push_back(descr.graphics);

		mCellIDS.count += descr.nTriangles;
		total_alloc += mem;
	}

	if (!InitBodyDescriptors()) {
		ERR("Unable to allocate body descriptors on device!");
		ShutdownDevice();
		return;
	}
	if (!InitCellIDS()) {
		ERR("Unable to init Cell IDS!");
		ShutdownDevice();
		return;
	}

	DBG("Allocated %ld bytes on device", total_alloc);
}

CUDAContext::~CUDAContext()
{
	ShutdownDevice();
}

void CUDAContext::UpdateVertexBuffers(bool async)
{
	cudaError_t err;
	MeshData::Vertex *ptr;
	int threadsPerBlock = 128;

	// map all in one call
	err = cudaGraphicsMapResources(mResArray.size(), &mResArray[0]);
	if (err != cudaSuccess) return;

	FOREACH(it, &mDescriptors) {
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

	cudaGraphicsUnmapResources(mResArray.size(), &mResArray[0]);
}

CUDASoftBodySolver::CUDASoftBodySolver(void)
	:
		mContext(0),
		mInitialized(false)
{
	mWorldParams.gravity = vec3(0, -10.0f, 0);
	mWorldParams.groundLevel = -2.0f;
}

CUDASoftBodySolver::~CUDASoftBodySolver(void)
{
	if (mContext) delete mContext;
}

bool CUDASoftBodySolver::initialize(void)
{
	if (mInitialized) return true;

	mContext = new CUDAContext(&mBodies);
		if (!mContext) {
		ERR("Unable to create CUDA context.");
		return false;
	}

	mBodies.clear();
	mInitialized = true;
	return true;
}

void CUDASoftBodySolver::shutdown(void)
{
	if (!mInitialized) return;

	if (mContext) {
		delete mContext;
		mContext = NULL;
	}
	mInitialized = false;
}

void CUDASoftBodySolver::updateVertexBuffersAsync(void)
{
	if (mInitialized)
		mContext->UpdateVertexBuffers(true);
}

void CUDASoftBodySolver::updateVertexBuffers(void)
{
	if (mInitialized)
		mContext->UpdateVertexBuffers(false);
}

void CUDAContext::ProjectSystem(float_t dt, CUDASoftBodySolver::SoftBodyWorldParameters &world)
{
	int threadsPerBlock = 128;
	int blockCount;
	int linkBlockCount;
	int idx = 0;

	// predict motion
	FOREACH(it, &mDescriptors) {
		blockCount = it->nParticles / threadsPerBlock + 1;

		cudaProjectPositionsAndVelocitiesKernel<<<blockCount,
			threadsPerBlock>>>(world.gravity, it->positions,
				it->projections, it->velocities, it->forces, it->massesInv, dt,
				it->nParticles);
	}

	// collision detection
	FOREACH(it, &mDescriptors) {
		blockCount = it->nTriangles / threadsPerBlock + 1;
		calculateSpatialHash<<<blockCount, threadsPerBlock>>>(
				idx, it->baseIdx, it->triangles, it->projections,
				DEFAULT_CELL_SIZE, mCellIDS.devPtr, it->nTriangles);
		idx++;
	}

	// solver
	FOREACH(it, &mDescriptors) {
		linkBlockCount = it->nLinks / MAX_LINKS + 1;
		blockCount = it->nParticles / threadsPerBlock + 1;

		for (int i = 0; i < mSolverSteps; i++) {
			solveLinksConstraints<<<linkBlockCount, threadsPerBlock>>>(
					1, it->links, it->projections, it->massesInv, it->nLinks);
			solveCollisionConstraints<<<blockCount, threadsPerBlock>>>(
					it->projections, it->massesInv,
					world.groundLevel, it->nParticles);
		}
	}

	// integrate motion
	FOREACH(it, &mDescriptors) {
		threadsPerBlock = 128;
		blockCount = it->nParticles / threadsPerBlock + 1;
		integrateMotionKernel<<<blockCount, threadsPerBlock>>>(dt, it->positions, it->projections,
				it->velocities, it->nParticles);
	}
}

void CUDASoftBodySolver::projectSystem(float_t dt)
{
	if (mInitialized)
		mContext->ProjectSystem(dt, mWorldParams);
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
