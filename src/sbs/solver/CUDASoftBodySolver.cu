#include "common.h"

#include "sbs/solver/CUDASoftBodySolver.h"
#include "sbs/solver/CUDAVector.h"
#include "sbs/solver/Math.h"
#include "sbs/solver/CUDASoftBodySolverKernel.h"

#include <cstring>
#include <set>
#include <queue>

using namespace std;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#define DEFAULT_SOLVER_STEPS 10

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
	void CreateDescriptor(SoftBody *body);
	void CreateShapeDescriptor(SoftBody *body);
	bool RegisterVertexBuffers(SoftBodyDescriptor &descr);
	cudaGraphicsResource *RegisterGLGraphicsResource(const VertexBuffer *vb);

	int                                mDeviceId;
	cudaDeviceProp                     mDevProp;
	cudaStream_t                       mStream;
	int                                mSolverSteps;

	descriptorArray_t                  mDescriptors;
	CUDAVector<SoftBodyDescriptor>	   mDescriptorsDev;
	CUDAVector<ShapeDescriptor>        mShapeDescriptors;

	// shape matching
	CUDAVector<ShapeRegionStaticInfo>  mRegions;
	CUDAVector<ShapeRegionDynamicInfo> mRegionsDynamicInfo;
	CUDAVector<glm::uint_t>            mRegionsMembersOffsets;
	CUDAVector<glm::uint_t>            mMembersRegionsOffsets;
	CUDAVector<glm::vec3>              mShapeInitialPositions; // initial particle locations (x0i);
	CUDAVector<glm::float_t>           mPartials;

	CUDAVector<ParticleInfo>           mParticlesInfo;
	CUDAVector<glm::vec3>              mPositions;
	CUDAVector<glm::vec3>              mProjections;
	CUDAVector<glm::vec3>              mVelocities;
	CUDAVector<glm::float_t>           mInvMasses;
	CUDAVector<glm::vec3>              mForces;

	CUDAVector<LinkConstraint>         mLinks;
	CUDAVector<glm::uint_t>            mMapping;
	CUDAVector<glm::uvec3>             mTriangles;
	CUDAVector<glm::vec3>              mTrianglesNormals;
	CUDAVector<ParticleTrianglesInfo>  mParticleTriangleInfo;
	CUDAVector<glm::uint_t>            mParticleTriangleIndexes;

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

struct Node {
	Node(int i, int d) : idx(i), distance(d) {}
	int idx;
	int distance;
};

void GetRegion(int idx, const MeshData::neighboursArray_t &nei, int max, indexArray_t &out)
{
	std::queue<Node> toprocess;
	std::set<int> processed;

	toprocess.push(Node(idx, 0));

	while (!toprocess.empty()) {
		Node n = toprocess.front();
		if (processed.find(n.idx) == processed.end()) {
			out.push_back(n.idx);
			processed.insert(n.idx);
		}
		toprocess.pop();

		if (n.distance >= max) continue;

		FOREACH_R(it, nei[n.idx])
			toprocess.push(Node(*it, n.distance + 1));
	}
}

void CUDAContext::CreateShapeDescriptor(SoftBody *obj)
{
	ShapeDescriptor d;
	vec3Array_t initQ;
	long len = 0;
	unsigned int smin = 999999;
	unsigned int smax = 0;
	int region_size = 2;

	d.mc0 = calculateMassCenter(
			&(obj->mParticles[0]), &(obj->mMassInv[0]), obj->mParticles.size());

	d.initPosBaseIdx = mShapeInitialPositions.size(); // shuld depend on mesh
	mShapeInitialPositions.push_back(&(obj->mParticles[0]), obj->mParticles.size());

	d.radius = 0;
	const MeshData::neighboursArray_t &na = obj->mMesh->GetNeighboursArray();

	int regions_base_id = mRegions.size();

	std::vector< std::vector<glm::uint_t> > particlesInRegions;
	particlesInRegions.resize(obj->mParticles.size());

	REP(i, obj->mParticles.size()) {
		indexArray_t indexes;
		GetRegion(i, na, region_size, indexes);
		REP(p, indexes.size()) {
			particlesInRegions[indexes[p]].push_back(i);
		}
	}
	// triangles info
	std::vector< std::set<glm::uint_t> > particlesInTriangles;
	std::vector< std::vector<glm::uint_t> > particlesInTriangles2;
	particlesInTriangles.resize(obj->mParticles.size());
	particlesInTriangles2.resize(obj->mParticles.size());
	REP(i, obj->mTriangles.size()) {
		glm::uvec3 idxs = obj->mTriangles[i];
		particlesInTriangles[idxs[0]].insert(i);
		particlesInTriangles[idxs[1]].insert(i);
		particlesInTriangles[idxs[2]].insert(i);
	}
	REP(i, particlesInTriangles.size()) {
		FOREACH_R(it, particlesInTriangles[i])
			particlesInTriangles2[i].push_back(*it);
	}

#if 0
	eEP(i, particlesInRegions.size()) {
		printf("%d:", particlesInRegions[i].size());
		REP(j, particlesInRegions[i].size()) {
			printf("%d,", particlesInRegions[i][j]);
		}
		printf("\n");
	}
#endif
	// create shape regions
	REP(i, obj->mParticles.size()) {
		ShapeRegionStaticInfo reg;
		ParticleInfo info;
		info.region_id = mRegions.size();
		info.body_info_id = mShapeDescriptors.size();
		info.body_offset = mPositions.size();
		indexArray_t indexes;
		float_t mass = 0.0f;
		glm::vec3 mc(0,0,0);

		GetRegion(i, na, region_size, indexes);

		len += indexes.size();
		if (smin > indexes.size())
			smin = indexes.size();
		if (smax < indexes.size())
			smax = indexes.size();

		FOREACH_R(it, indexes) {
			mass += obj->mMassInv[*it];
			mc += obj->mParticles[*it] * obj->mMassInv[*it];
		}
		reg.mass = mass;
		reg.mc0 = mc / mass;
		reg.n_particles = indexes.size();
		reg.members_offsets_offset = mRegionsMembersOffsets.size();
		reg.shapes_init_positions_offset = d.initPosBaseIdx;
		reg.regions_offsets_offset = mMembersRegionsOffsets.size();
		reg.n_regions = particlesInRegions[i].size();

		ParticleTrianglesInfo pti;
		pti.triangle_id_offset = mParticleTriangleIndexes.size();
		pti.n_triangles = particlesInTriangles2[i].size();

		mRegions.push_back(reg);
		mRegionsMembersOffsets.push_back(&indexes[0], indexes.size());
		mParticlesInfo.push_back(info);
		mMembersRegionsOffsets.push_back(&(particlesInRegions[i][0]),
				particlesInRegions[i].size());
		mParticleTriangleInfo.push_back(pti); 
		mParticleTriangleIndexes.push_back(&(particlesInTriangles2[i][0]),
				particlesInTriangles2[i].size());
	}

	DBG("==MODEL INFORMATION==");
	DBG("Particles total: %ld", obj->mParticles.size());
	DBG("Vertexes total: %ld", obj->mMesh->GetVertexes().size());
	DBG("Triangles total: %ld", obj->mMesh->GetFaces().size());
	DBG("Rest Volume :%f", d.volume);
	DBG("Regions total: %ld", mRegions.size());
	DBG("Average region size: %f", (float)len / mRegions.size());
	DBG("Max region size: %d", smax);
	DBG("Min region size: %d", smin);

	DBG("ParticeInfo size: %d", mParticlesInfo.size());

	mShapeDescriptors.push_back(d);
}

void CUDAContext::CreateDescriptor(SoftBody *body)
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

	descr.volume0 = calculateVolume(&(body->mParticles[0]),
			&(body->mTriangles[0]), NULL, NULL, body->mTriangles.size()); 

	bool res = RegisterVertexBuffers(descr);
	if (!res) {
		ERR("Error occured registering SoftBody vertex buffers.");
		return;
	}

	mDescriptors.push_back(descr);
	mDescriptorsDev.push_back(descr);
	mPartials.resize(body->mTriangles.size() / 128 + 1);
	mResArray.push_back(descr.graphics);
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
	//int blocks = descr.nLinks / 128;
	//calculateLinkStiffness<<<blocks, 128>>>(mSolverSteps, mLinks.data(),
	//		descr.linkIdx, descr.nLinks);
}

bool CUDAContext::InitSoftBody(SoftBody *body)
{
	CreateDescriptor(body);
	CreateShapeDescriptor(body);

	long nParticles = body->mParticles.size();
	mPositions.push_back(&(body->mParticles[0]), nParticles);
	mProjections.push_back(&(body->mParticles[0]), nParticles);
	mVelocities.resize(mVelocities.size() + nParticles);
	mInvMasses.push_back(&(body->mMassInv[0]), nParticles);
	mForces.resize(mForces.size() + nParticles);
	mLinks.push_back(&(body->mLinks[0]), body->mLinks.size());
	mMapping.push_back(&(body->mMeshVertexParticleMapping[0]),
			body->mMeshVertexParticleMapping.size());
	mRegionsDynamicInfo.resize(mPositions.size());
	mTriangles.push_back(&(body->mTriangles[0]), body->mTriangles.size());
	mTrianglesNormals.resize(body->mTriangles.size());

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
	solveShapeMatchingConstraints1<<<blockCount, threadsPerBlock>>>(
			mParticlesInfo.data(),
			mRegions.data(),
			mRegionsMembersOffsets.data(),
			mShapeInitialPositions.data(),
			mProjections.data(),
			mInvMasses.data(),
			mRegionsDynamicInfo.data(),
			mPositions.size()
			);

	solveShapeMatchingConstraints2<<<blockCount, threadsPerBlock>>>(
			mParticlesInfo.data(),
			mRegions.data(),
			mRegionsDynamicInfo.data(),
			mRegionsMembersOffsets.data(),
			mShapeInitialPositions.data(),
			mMembersRegionsOffsets.data(),
			mProjections.data(),
			mPositions.size()
			);

	blockCount = mTriangles.size() / threadsPerBlock + 1;
	solveVolumePreservationConstraint1<<<blockCount, threadsPerBlock>>>(
			mParticlesInfo.data(),
			mPartials.data(),
			mProjections.data(),
			mTriangles.data(),
			mTrianglesNormals.data(),
			mTriangles.size()
			);

	blockCount = mPartials.size() / threadsPerBlock + 1;
	solveVolumePreservationConstraint2<<<blockCount, threadsPerBlock>>>(
			mDescriptorsDev.data(),
			mPartials.data(),
			mPartials.size()
			);

	blockCount = mPositions.size() / threadsPerBlock + 1;
    solveVolumePreservationConstraint3<<<blockCount, threadsPerBlock>>>(
		mParticleTriangleInfo.data(),
		mDescriptorsDev.data(),
		mTrianglesNormals.data(),
		mProjections.data(),
		mParticleTriangleIndexes.data(),
		mPositions.size());

	solveGroundWallCollisionConstraints<<<blockCount, threadsPerBlock>>>(
			mProjections.data(), mInvMasses.data(),
			world.groundLevel, world.leftWall, world.rightWall,
			world.frontWall, world.backWall, mPositions.size());

#if 0
	vector<SoftBodyDescriptor> info;
	info.resize(1);
	cudaMemcpy(&info[0], mDescriptorsDev.data(), 
			sizeof(SoftBodyDescriptor),
			cudaMemcpyDeviceToHost);
	ERR("%f", info[0].volume);
#endif

#if 0
	vector<glm::float_t> info;
	info.resize(mPartials.size());
	cudaMemcpy(&info[0], mPartials.data(), 
			sizeof(glm::float_t) * mPartials.size(),
			cudaMemcpyDeviceToHost);
	float sum = 0;
	FOREACH_R(it, info) sum += *it;
	ERR("%f", sum);
#endif

#if 0
	vector<glm::mat3> info;
	info.resize(mTest.size());
	cudaMemcpy(&info[0], mTest.data(),
			sizeof(glm::mat3) * mTest.size(),
			cudaMemcpyDeviceToHost);
	vector<glm::vec3> info2;
	info2.resize(mTest2.size());
	cudaMemcpy(&info2[0], mTest2.data(),
			sizeof(glm::vec3) * mTest2.size(),
			cudaMemcpyDeviceToHost);

	REP(k, info.size()) {
		glm::mat3 R = info[k];
		glm::vec3 mc = info2[k];
		ERR("[%f %f %f]", mc[0], mc[1], mc[2]);
		ERR("[%f %f %f %f %f %f %f %f %f]", R[0][0], R[1][0], R[2][0],
				R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2]);
	}
#endif

#if 0
	vector<glm::uint> info;
	info.resize(mMembersRegionsOffsets.size());
	cudaMemcpy(&info[0], mMembersRegionsOffsets.data(),
			sizeof(glm::uint) * mMembersRegionsOffsets.size(),
			cudaMemcpyDeviceToHost);

	FOREACH_R(i, info) {
		ERR("%d,", *i);
	}
#endif

	// integrate motion
	threadsPerBlock = 128;
	blockCount = mPositions.size() / threadsPerBlock + 1;
	integrateMotionKernel<<<blockCount, threadsPerBlock>>>(
			dt, mPositions.data(), mProjections.data(),
			mVelocities.data(), mPositions.size());
	cudaDeviceSynchronize();

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
