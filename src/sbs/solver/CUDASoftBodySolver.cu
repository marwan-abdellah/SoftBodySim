#include "common.h"

#include "sbs/solver/CUDASoftBodySolver.h"
#include "sbs/solver/CUDAVector.h"
#include "sbs/solver/CUDASoftBodySolverKernel.h"

#include <cstring>

using namespace std;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#define DEFAULT_SOLVER_STEPS 10
#define DEFAULT_CELL_SIZE 1.0
#define MAX_REGION_SIZE 10

struct ShapeRegion {
	glm::vec3 mc0; // initial region mass center
	float_t mass; // region total mass
	glm::uint_t indexes[MAX_REGION_SIZE]; // indexes of region particles
};

struct ShapeDescriptor {
	glm::vec3 mc0; // body global mass
	CUDAVector<glm::vec3> initPos; // initial particle locations (x0i);
	float_t radius; // maximum distance between mass center and particle;
	float_t massTotal; // object total mass
	float_t volume; // initial shape volume
	CUDAVector<ShapeRegion> regions; // shape regions
};

class CUDAContext {
public:
	typedef std::vector<SoftBodyDescriptor> descriptorArray_t;

	//cudaContextCreate(softbodyList_t*);
	CUDAContext(softbodyList_t *list);

	//void cudaContextShutdown(SolverPrivate*);
	~CUDAContext(void);

	bool InitDevice();
	bool ShutdownDevice();

	void UpdateVertexBuffers(bool async);
	void ProjectSystem(glm::float_t dt, CUDASoftBodySolver::SoftBodyWorldParameters
			&parms);
	bool InitSoftBody(SoftBody *body);
private:
	void UpdateConstraintStiffness(SoftBodyDescriptor &descr, int mSolverSteps);
	SoftBodyDescriptor CreateDescriptor(SoftBody *body);
	bool RegisterVertexBuffers(SoftBodyDescriptor &descr);
	cudaGraphicsResource *RegisterGLGraphicsResource(const VertexBuffer *vb);

	int                                mDeviceId;
	cudaDeviceProp                     mDevProp;
	cudaStream_t                       mStream;
	int                                mSolverSteps;

	descriptorArray_t                  mDescriptors;
	CUDAVector<SoftBodyDescriptor>	   mDescriptorsDev;
	CUDAVector<ShapeDescriptor>        mShapeDescriptors;

	CUDAVector<glm::vec3>              mPositions;
	CUDAVector<glm::vec3>              mProjections;
	CUDAVector<glm::vec3>              mVelocities;
	CUDAVector<glm::float_t>           mInvMasses;
	CUDAVector<glm::vec3>              mForces;

	CUDAVector<LinkConstraint>         mLinks;
	CUDAVector<glm::uint_t>            mMapping;
	CUDAVector<glm::uvec3>             mTriangles;

	vector<cudaGraphicsResource*>      mResArray; /* helper array to map all resources 
												  in one call */
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

	err = cudaDeviceSynchronize();
	if (err != cudaSuccess) return false;

	err = cudaDeviceReset();
	if (err != cudaSuccess) return false;


	return true;
}

SoftBodyDescriptor CUDAContext::CreateDescriptor(SoftBody *body)
{
	SoftBodyDescriptor descr;

	descr.body = body;
	descr.graphics = NULL;
	descr.baseIdx = mPositions.size();
	descr.nParticles = body->mParticles.size();
	descr.linkIdx = mLinks.size();
	descr.nLinks = body->mLinks.size();
	descr.mappingIdx = mMapping.size();
	descr.nMapping = body->mMeshVertexParticleMapping.size();
	descr.trianglesIdx = mTriangles.size();
	descr.nTriangles = body->mTriangles.size();

	return descr;
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

bool CUDAContext::RegisterVertexBuffers(SoftBodyDescriptor &descr)
{
	if (!descr.body) {
		ERR("No SoftBody reference in descriptor!");
		return false;
	}

	const VertexBuffer *buf = descr.body->GetVertexes();
	if (buf)
		descr.graphics = RegisterGLGraphicsResource(buf);

	return true;
}

void CUDAContext::UpdateConstraintStiffness(SoftBodyDescriptor
		&descr, int mSolverSteps)
{
	int blocks = descr.nLinks / 128;
	calculateLinkStiffness<<<blocks, 128>>>(mSolverSteps, mLinks.data(),
			descr.linkIdx, descr.nLinks);
}

bool CUDAContext::InitSoftBody(SoftBody *body)
{
	SoftBodyDescriptor descr = CreateDescriptor(body);
	bool res;

	mPositions.push_back(&(body->mParticles[0]), descr.nParticles);
	mProjections.push_back(&(body->mParticles[0]), descr.nParticles);
	mVelocities.resize(mVelocities.size() + descr.nParticles);
	mInvMasses.push_back(&(body->mMassInv[0]), descr.nParticles);
	mForces.resize(mForces.size() + descr.nParticles);
	mLinks.push_back(&(body->mLinks[0]), descr.nLinks);
	mMapping.push_back(&(body->mMeshVertexParticleMapping[0]), descr.nMapping);

	res = RegisterVertexBuffers(descr);
	if (!res) {
		ERR("Error occured registering SoftBody vertex buffers.");
		ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
		return false;
	}

	UpdateConstraintStiffness(descr, mSolverSteps);

	mDescriptors.push_back(descr);
	mDescriptorsDev.push_back(descr);
	mResArray.push_back(descr.graphics);

	return true;
}

#if 0
bool CUDAContext::InitDymmyBodyCollisionConstraint()
{
	long int total = 0, bytes = 0;
	vector<PointTriangleConstraint> constraints;
	PointTriangleConstraint con;

	// constant collision handling
	// create m * x collsion constraints - to be optimized later.
	FOREACH(it, &mDescriptors) {
		constraints.clear();
		FOREACH(vx, &it->body->mPositions) {
			int idx = std::distance(it->body->mPositions.begin(), vx);
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
#endif

CUDAContext::CUDAContext(softbodyList_t *bodies)
{
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

#if 0
	if (!InitDymmyBodyCollisionConstraint()) {
		ERR("Unable to allocate collision constraints on device!");
		ShutdownDevice();
		return;
	}
#endif 
}

CUDAContext::~CUDAContext()
{
	ShutdownDevice();
}

void CUDAContext::UpdateVertexBuffers(bool async)
{
	cudaError_t err;
	glm::vec3 *ptr;
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
				ptr, mPositions.data(), mMapping.data(), it->baseIdx, 
				it->mappingIdx, it->nMapping);
	}

	cudaGraphicsUnmapResources(mResArray.size(), &mResArray[0]);
}

CUDASoftBodySolver::CUDASoftBodySolver(void)
	:
		mContext(0),
		mInitialized(false)
{
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
	mInitialized = false;
	SoftBodySolver::Shutdown();
}

void CUDASoftBodySolver::UpdateVertexBuffers(void)
{
	if (mInitialized)
		mContext->UpdateVertexBuffers(false);
}

void CUDAContext::ProjectSystem(glm::float_t dt, CUDASoftBodySolver::SoftBodyWorldParameters &world)
{
	int threadsPerBlock = 128;
	int blockCount;

	if (!mPositions.size()) return;

	// predict motion
	blockCount = mPositions.size() / threadsPerBlock + 1;
	cudaProjectPositionsAndVelocitiesKernel<<<blockCount,
		threadsPerBlock>>>(world.gravity, mPositions.data(),
			mProjections.data(), mVelocities.data(), NULL,
			mInvMasses.data(), dt, mPositions.size());

	// solver
	blockCount = mPositions.size() / threadsPerBlock + 1;
	FOREACH(it, &mDescriptors) {
		int linkBlockCount = it->nLinks / MAX_LINKS + 1;

		solveLinksConstraints<<<linkBlockCount, threadsPerBlock>>>(
				1, mLinks.data(), mProjections.data(), mInvMasses.data(),
				it->baseIdx, it->linkIdx, it->nLinks);
	}
	solveGroundWallCollisionConstraints<<<blockCount, threadsPerBlock>>>(
			mProjections.data(), mInvMasses.data(),
			world.groundLevel, world.leftWall, world.rightWall,
			world.frontWall, world.backWall, mPositions.size());

	// integrate motion
	threadsPerBlock = 128;
	blockCount = mPositions.size() / threadsPerBlock + 1;
	integrateMotionKernel<<<blockCount, threadsPerBlock>>>(
			dt, mPositions.data(), mProjections.data(),
			mVelocities.data(), mPositions.size());
}

void CUDASoftBodySolver::ProjectSystem(glm::float_t dt)
{
	if (mInitialized)
		mContext->ProjectSystem(dt, mWorldParams);
}

void CUDASoftBodySolver::AddSoftBody(SoftBody *body)
{
	mBodies.push_back(body);
	if (!mInitialized || !mContext->InitSoftBody(body))
		ERR("Failed to add SoftBody!");
}
