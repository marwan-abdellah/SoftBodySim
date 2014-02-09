#include "CUDASoftBodySolver.h"

#include "common.h"
#include <cstring>

using namespace std;
using namespace glm;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

enum ArrayType {
    ARRAY_POSITIONS = 0,
    ARRAY_PROJECTIONS,
    ARRAY_VELOCITIES,
    ARRAY_FORCES,
    ARRAY_LAST_DEFINED
};

struct CUDASoftBodySolver::SoftBodyDescriptor {
    SoftBody *body;
    cudaGraphicsResource *graphics;

    unsigned int vertexBaseIdx;
    unsigned int nVertex;

    unsigned int linksBaseIdx;
    unsigned int nLinks;

    unsigned int mappingBaseIdx;
    unsigned int nMapping;
};

struct CUDASoftBodySolver::SolverPrivate {
    int             deviceId;
    cudaDeviceProp  devProp;
    cudaStream_t    stream;

    vec3            *array[ARRAY_LAST_DEFINED];
    float_t         *massInv;
	unsigned int	nTotalVertex; /* number of elements in each array and massInv */

    uvec2           *links;
    float_t         *linksRestLength2;
	unsigned int	nTotalLinks;

    uint_t          *mapping;  /* For updates only. keeps ids of particle per vertexes in VertexBuffer */
	unsigned int	nTotalMapping;

	vector<SoftBodyDescriptor> descriptors;
};

CUDASoftBodySolver::CUDASoftBodySolver(void)
    : mInitialized(false)
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

bool CUDASoftBodySolver::cudaAllocateDeviceBuffers(SolverPrivate *cuda)
{
    int bytesArray = 0, bytesMass = 0, bytesMapping = 0, bytesLinks = 0, bytesRestLength = 0;
    long total_alloc = 0;

	cuda->nTotalVertex = cuda->nTotalLinks = cuda->nTotalMapping = 0;

    FOREACH(it, &cuda->descriptors) {
        cuda->nTotalVertex += it->nVertex;
        cuda->nTotalMapping += it->nMapping;
        cuda->nTotalLinks += it->nLinks;
    }

	bytesArray += cuda->nTotalVertex * sizeof(vec3);
	bytesMass += cuda->nTotalVertex * sizeof(float_t);
	bytesMapping += cuda->nTotalMapping * sizeof(uint_t);
	bytesLinks += cuda->nTotalLinks * sizeof(uvec2);
	bytesRestLength += cuda->nTotalLinks * sizeof(float_t);

    cuda->array[ARRAY_POSITIONS]   = (vec3*)allocateCUDABuffer(bytesArray);
    cuda->array[ARRAY_PROJECTIONS] = (vec3*)allocateCUDABuffer(bytesArray);
    cuda->array[ARRAY_FORCES]      = (vec3*)allocateCUDABuffer(bytesArray, true);
    cuda->array[ARRAY_VELOCITIES]  = (vec3*)allocateCUDABuffer(bytesArray);
    for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; ++type)
        if (!cuda->array[type]) goto on_fail;

    cuda->massInv = (float_t*)allocateCUDABuffer(bytesMass);
    if (!cuda->massInv) goto on_fail;

    cuda->mapping = (uint*)allocateCUDABuffer(bytesMapping);
    if (!cuda->mapping) goto on_fail;

    cuda->links = (uvec2*)allocateCUDABuffer(bytesLinks);
    if (!cuda->links) goto on_fail;

    cuda->linksRestLength2 = (float_t*)allocateCUDABuffer(bytesRestLength);
    if (!cuda->linksRestLength2) goto on_fail;

    total_alloc = ARRAY_LAST_DEFINED * bytesArray + bytesMapping + bytesMass + bytesLinks + bytesRestLength;
    DBG("Device allocated mem: %ld bytes", total_alloc);

    return true;

on_fail:
    cudaDeallocateDeviceBuffers(cuda);
    return false;
}

void CUDASoftBodySolver::cudaDeallocateDeviceBuffers(SolverPrivate *cuda)
{
    for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; ++type)
        if (cuda->array[type]) cudaFree(cuda->array[type]);
    if (cuda->massInv) cudaFree(cuda->massInv);
    if (cuda->links) cudaFree(cuda->links);
    if (cuda->linksRestLength2) cudaFree(cuda->linksRestLength2);
    if (cuda->mapping) cudaFree(cuda->mapping);
}

void CUDASoftBodySolver::cudaCreateDescriptors(SolverPrivate *cuda, softbodyArray_t *bodies)
{
    int idx1 = 0, idx2= 0, idx3 = 0;

	cuda->descriptors.clear();

    FOREACH(it, bodies) {
        SoftBodyDescriptor descr;
        SoftBody *body = *it;
        descr.body = *it;
        descr.vertexBaseIdx = idx1;
        descr.linksBaseIdx = idx2;
        descr.mappingBaseIdx = idx3;

        descr.nVertex = body->mParticles.size();
        descr.nLinks = body->mLinks.size();
        descr.nMapping = body->mMeshVertexParticleMapping.size();

        cuda->descriptors.push_back(descr);

        idx1 += descr.nVertex;
        idx2 += descr.nMapping;
        idx3 += descr.nLinks;
    }
}

bool CUDASoftBodySolver::cudaCopyBodyToDeviceBuffers(SoftBodyDescriptor *descr, SolverPrivate *cuda)
{
    cudaError_t err;

    SoftBody *body = descr->body;

    vector<uvec2> tmp(body->mLinks.size());
    vector<float_t> tmp2(body->mLinks.size());

    unsigned int bytes1 = descr->nVertex * sizeof(vec3);
    unsigned int bytes2 = descr->nLinks * sizeof(uvec2);
    unsigned int bytes3 = descr->nLinks * sizeof(float_t);
    unsigned int bytes4 = descr->nVertex * sizeof(float_t);
    unsigned int bytes5 = descr->nMapping * sizeof(uint_t);

    unsigned int offset1 = descr->vertexBaseIdx * sizeof(vec3);
    unsigned int offset2 = descr->linksBaseIdx * sizeof(uvec2);
    unsigned int offset3 = descr->linksBaseIdx * sizeof(float_t);
    unsigned int offset4 = descr->vertexBaseIdx * sizeof(float_t);
    unsigned int offset5 = descr->mappingBaseIdx * sizeof(uint_t);

    unsigned char *ptr;

    ptr = reinterpret_cast<unsigned char*>(cuda->array[ARRAY_POSITIONS]);
    ptr += offset1;
    err = cudaMemcpy(ptr, &(body->mParticles[0]), bytes1, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) return false;

    ptr = reinterpret_cast<unsigned char*>(cuda->array[ARRAY_PROJECTIONS]);
    ptr += offset1;
    err = cudaMemcpy(ptr, &(body->mParticles[0]), bytes1, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) return false;

    ptr = reinterpret_cast<unsigned char*>(cuda->array[ARRAY_VELOCITIES]);
    ptr += offset1;
    err = cudaMemcpy(ptr, &(body->mVelocities[0]), bytes1, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) return false;

    ptr = reinterpret_cast<unsigned char*>(cuda->array[ARRAY_FORCES]);
    ptr += offset1;
    err = cudaMemcpy(ptr, &(body->mForces[0]), bytes1, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) return false;

    ptr = reinterpret_cast<unsigned char*>(cuda->mapping);
    ptr += offset5;
    err = cudaMemcpy(ptr, &(body->mMeshVertexParticleMapping[0]), bytes5, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) return false;

    ptr = reinterpret_cast<unsigned char*>(cuda->massInv);
    ptr += offset4;
    err = cudaMemset(ptr, 0x0, bytes4);
    if (err != cudaSuccess) return false;

    FOREACH_R(lnk, body->mLinks)
    {
        tmp.push_back(lnk->index);
        tmp2.push_back(lnk->restLength);
    }

    ptr = reinterpret_cast<unsigned char*>(cuda->links);
    ptr += offset2;
    err = cudaMemcpy(ptr, &tmp[0], bytes2, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) return false;

    ptr = reinterpret_cast<unsigned char*>(cuda->linksRestLength2);
    ptr += offset3;
    err = cudaMemcpy(ptr, &tmp2[0], bytes3, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) return false;

    return true;
}

bool CUDASoftBodySolver::cudaRegisterGLGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr)
{
    cudaError_t err;
    GLuint id = vb->getVBO(GLVertexBuffer::VERTEX_ATTR_POSITION);
    err = cudaGraphicsGLRegisterBuffer(&descr->graphics, id, cudaGraphicsRegisterFlagsNone);
    if (err != cudaSuccess) {
        ERR("Unable to register GL buffer object %d", id);
        descr->graphics = NULL;
        return false;
    }
    return true;
}

bool CUDASoftBodySolver::cudaRegisterVertexBuffers(SoftBodyDescriptor *descr, SolverPrivate *cuda)
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
                if (!cudaRegisterGLGraphicsResource(static_cast<const GLVertexBuffer*>(buf), descr))
                break;
            default:
                break;
        }
    }

    return true;
}

bool CUDASoftBodySolver::cudaInitializeBodies(SolverPrivate *cuda)
{
	bool res;
    FOREACH(it, &cuda->descriptors) {
        res = cudaCopyBodyToDeviceBuffers(&(*it), cuda);
        if (!res) {
            ERR("Error occured while copying Soft bodies data to device!");
            ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
            return false;
        }
        res = cudaRegisterVertexBuffers(&(*it), cuda);
        if (!res) {
            ERR("Error occured registering SoftBody vertex buffers.");
            ERR("Cuda error: %s", cudaGetErrorString(cudaGetLastError()));
            return false;
        }
	}
	return true;
}

CUDASoftBodySolver::SolverPrivate *CUDASoftBodySolver::cudaContextCreate(void)
{
    SolverPrivate *cuda;

    cuda = new SolverPrivate;
    memset(cuda, 0x0, sizeof(SolverPrivate));

    if (!cudaInitializeDevice(cuda)) {
        ERR("CUDA Device initialization failed!");
		delete cuda;
        return NULL;
    }

	return cuda;
}

void CUDASoftBodySolver::cudaShutdown(SolverPrivate *cuda)
{
    cudaDeallocateDeviceBuffers(cuda);
    cudaShutdownDevice(cuda);
}

bool CUDASoftBodySolver::cudaInitialize(SolverPrivate *cuda, softbodyArray_t *bodies)
{
    cudaCreateDescriptors(cuda, bodies);

    if (!cudaAllocateDeviceBuffers(cuda)) {
        ERR("Unable to allocte enough memory on device!");
        return false;
    }

	if (!cudaInitializeBodies(cuda)) {
        ERR("Unable to allocte enough memory on device!");
		cudaDeallocateDeviceBuffers(cuda);
        return false;
    }
	return true;
}

bool CUDASoftBodySolver::initialize(softbodyArray_t *bodies)
{
    SolverPrivate *cuda;

    if (mInitialized) return true;

	cuda = cudaContextCreate();
	if (!cuda) {
		ERR("Unable to create CUDA context.");
		return false;
	}

	if (!cudaInitialize(cuda, bodies)) {
		ERR("Unable to initailize cuda resources");
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
		cudaShutdown(mCuda);
        mCuda = NULL;
    }
    mInitialized = false;
}

void CUDASoftBodySolver::updateVertexBuffers(SolverPrivate *cuda, bool async = false)
{
	cudaError_t err;
	size_t nRes;
	vec3 *ptr;

	nRes = cuda->descriptors.size();
	cudaGraphicsResource **res = new cudaGraphicsResource*[nRes];
	for (unsigned int i = 0; i < nRes; i++)
		res[i] = cuda->descriptors[i].graphics;

	err = cudaGraphicsMapResources(nRes, res);
	if (err != cudaSuccess) {
		delete res;
		return;
	}

	FOREACH(it, &cuda->descriptors) {
		size_t size;
		err = cudaGraphicsResourceGetMappedPointer((void**)&ptr, &size, it->graphics);
		if (err != cudaSuccess) {
			ERR("Unable to map VBO pointer");
			delete res;
			return;
		}
		if (size != it->nVertex * sizeof(vec3)) {
			ERR("Invalid size!");
			delete res;
			return;
		}
		cudaUpdateVertexBuffer(cuda->array[ARRAY_POSITIONS], cuda->mapping, ptr, it->vertexBaseIdx, it->nVertex);
	}

	cudaGraphicsUnmapResources(nRes, res);
	delete res;
}

void CUDASoftBodySolver::updateVertexBuffersAsync(void)
{
	if (mInitialized)
		updateVertexBuffers(mCuda, true);
}

void CUDASoftBodySolver::updateVertexBuffers(void)
{
	if (mInitialized)
		updateVertexBuffers(mCuda, false);
}

void CUDASoftBodySolver::projectSystem(float_t dt)
{
}
