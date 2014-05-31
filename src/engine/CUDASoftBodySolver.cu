#include "CUDASoftBodySolver.h"
#include "CUDAVector.h"

#include "common.h"
#include <cstring>

using namespace std;
using namespace glm;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "CUDASoftBodySolverKernel.h"

#define DEFAULT_SOLVER_STEPS 15
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
	bool InitDymmyBodyCollisionConstraint();
	bool InitSoftBody(SoftBody *body);
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
	if (descr->collisions)  cudaFree(descr->collisions);
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
	unsigned int bytesMap  = descr->nMapping * sizeof(uint_t);
	unsigned int bytesTria = descr->nTriangles * sizeof(uvec3);
	unsigned int bytesMass = descr->nParticles * sizeof(float_t);

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

bool CUDAContext::InitSoftBody(SoftBody *body)
{
	SoftBodyDescriptor descr = CreateDescriptor(body);
	bool res;

	long mem = AllocateDeviceBuffers(&descr);
	if (mem == -1) {
		ERR("Unable to allocate memory for SoftBody");
		return false;
	}
	res = CopyBodyToDeviceBuffers(&descr);
	if (!res) {
		ERR("Error occured while copying Soft bodies data to device!");
		ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
		return false;
	}
	res = RegisterVertexBuffers(&descr);
	if (!res) {
		ERR("Error occured registering SoftBody vertex buffers.");
		ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
		return false;
	}
	UpdateConstraintStiffness(&descr, mSolverSteps);

	mDescriptors.push_back(descr);
	mResArray.push_back(descr.graphics);
	mCellIDS.count += descr.nTriangles;

	return true;
}

bool CUDAContext::InitDymmyBodyCollisionConstraint()
{
	long int total = 0, bytes = 0;
	vector<PointTriangleConstraint> constraints;
	PointTriangleConstraint con;

	// constant collision handling
	// create m * x collsion constraints - to be optimized later.
	FOREACH(it, &mDescriptors) {
		constraints.clear();
		FOREACH(vx, &it->body->mParticles) {
			int idx = std::distance(it->body->mParticles.begin(), vx);
			con.pointObjectId = std::distance(mDescriptors.begin(), it);
			con.pointIdx = idx;
			/*
			FOREACH(tr, &it->body->mTriangles) {
				if (idx == (*tr)[0] ||
					idx == (*tr)[1] ||
					idx == (*tr)[2]) continue;
				con.triangleObjectId = std::distance(mDescriptors.begin(), it);
				con.triangleId = std::distance(it->body->mTriangles.begin(), tr);
				constraints.push_back(con);
			}
			*/
			FOREACH(it2, &mDescriptors) {
				if (it == it2) continue;
				FOREACH(tr, &it->body->mTriangles) {
					con.triangleObjectId = std::distance(
							mDescriptors.begin(), it2);
					con.triangleId = std::distance(it->body->mTriangles.begin(),
							tr);
					constraints.push_back(con);
					total++;
				}
			}
		}
		if (it->collisions) cudaFree(it->collisions);
		it->collisions = (PointTriangleConstraint*)allocateCUDABuffer(sizeof(PointTriangleConstraint) * constraints.size());
		it->nCollisions = constraints.size();
		cudaMemcpy(it->collisions, &constraints[0],
				sizeof(PointTriangleConstraint) * constraints.size(),
				cudaMemcpyHostToDevice);
		bytes += sizeof(PointTriangleConstraint) * constraints.size();
	}
	DBG("allocated constraints %d, bytes %d", total, bytes);
	return true;
}

bool CUDAContext::InitBodyDescriptors()
{
	cudaError_t err;
	size_t bytes = sizeof(SoftBodyDescriptor) * mDescriptors.size();

	if (!mDescriptors.size()) return true;
	if (mDescriptorsDev) cudaFree(mDescriptorsDev);

	mDescriptorsDev = (SoftBodyDescriptor*)allocateCUDABuffer(bytes);
	if (!mDescriptorsDev) return false;

	err = cudaMemcpy(mDescriptorsDev, &mDescriptors[0], bytes, cudaMemcpyHostToDevice);
	if (err != cudaSuccess) return false;

	return true;
}

bool CUDAContext::InitCellIDS()
{
	// currently not used.
	return true;
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
	mCellIDS.count = 0;
	mSolverSteps = DEFAULT_SOLVER_STEPS;

	if (!InitDevice()) {
		ERR("CUDA Device initialization failed!");
		return;
	}

	FOREACH(it, bodies) {
		if (!*it) continue;

		if (!InitSoftBody(*it)) {
			ShutdownDevice();
			return;
		}
	}

	if (!InitBodyDescriptors()) {
		ERR("Unable to allocate body descriptors on device!");
		ShutdownDevice();
		return;
	}

#if 0
	if (!InitDymmyBodyCollisionConstraint()) {
		ERR("Unable to allocate collision constraints on device!");
		ShutdownDevice();
		return;
	}
#endif 
	if (!InitCellIDS()) {
		ERR("Unable to init Cell IDS!");
		ShutdownDevice();
		return;
	}
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

bool CUDASoftBodySolver::Initialize(void)
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

void CUDASoftBodySolver::Shutdown(void)
{
	if (!mInitialized) return;

	if (mContext) {
		delete mContext;
		mContext = NULL;
	}
	mBodies.clear();
	mInitialized = false;
}

void CUDASoftBodySolver::UpdateVertexBuffers(void)
{
	if (mInitialized)
		mContext->UpdateVertexBuffers(false);
}

void CUDAContext::ProjectSystem(float_t dt, CUDASoftBodySolver::SoftBodyWorldParameters &world)
{
	int threadsPerBlock = 128;
	int blockCount;
	int linkBlockCount, collBlockCount;

	// predict motion

	FOREACH(it, &mDescriptors) {
		blockCount = it->nParticles / threadsPerBlock + 1;
		cudaProjectPositionsAndVelocitiesKernel<<<blockCount,
			threadsPerBlock>>>(world.gravity, it->positions,
				it->projections, it->velocities, it->forces, it->massesInv, dt,
				it->nParticles);
	}

	// collision detection
#if 0
	FOREACH(it, &mDescriptors) {
		blockCount = it->nTriangles / threadsPerBlock + 1;
		calculateSpatialHash<<<blockCount, threadsPerBlock>>>(
				idx, it->baseIdx, it->triangles, it->projections,
				DEFAULT_CELL_SIZE, mCellIDS.devPtr, it->nTriangles);
		idx++;
	}

#endif
	// solver
	for (int i = 0; i < mSolverSteps; i++) {
#if 0
		FOREACH(it, &mDescriptors) {
			linkBlockCount = it->nLinks / MAX_LINKS + 1;
			blockCount = it->nParticles / threadsPerBlock + 1;
			collBlockCount = it->nCollisions / threadsPerBlock + 1;

			solveLinksConstraints<<<linkBlockCount, threadsPerBlock>>>(
<<<<<<< HEAD
					1, it->links, it->particles, it->nLinks);
//			solvePointTriangleCollisionsKernel<<<collBlockCount,
//				threadsPerBlock>>>(mDescriptorsDev, it->collisions,
//						it->nCollisions);
=======
					1, it->links, it->projections, it->massesInv, it->nLinks);
			solvePointTriangleCollisionsKernel<<<collBlockCount,
				threadsPerBlock>>>(mDescriptorsDev, it->collisions,
						it->nCollisions);
			solveCollisionConstraints<<<blockCount, threadsPerBlock>>>(
					it->projections, it->massesInv,
					world.groundLevel, it->nParticles);
>>>>>>> parent of 5f5f748... solver: introducet particle structure
		}
		solveGroundCollisionConstraints<<<blockCount, threadsPerBlock>>>(
				mParticles.data(), world.groundLevel, mParticles.size());
#endif
	}

	// integrate motion
	FOREACH(it, &mDescriptors) {
		threadsPerBlock = 128;
		blockCount = it->nParticles / threadsPerBlock + 1;
		integrateMotionKernel<<<blockCount, threadsPerBlock>>>(dt, it->positions, it->projections,
				it->velocities, it->nParticles);
	}
}

void CUDASoftBodySolver::ProjectSystem(float_t dt)
{
	if (mInitialized)
		mContext->ProjectSystem(dt, mWorldParams);
}

void CUDASoftBodySolver::AddSoftBody(SoftBody *body)
{
	mBodies.push_back(body);
	bool res = true;
	if (mInitialized) {
		res &= mContext->InitSoftBody(body);
		res &= mContext->InitBodyDescriptors();
		if (!res)
			ERR("Failed to add SoftBody!");
	}
}
