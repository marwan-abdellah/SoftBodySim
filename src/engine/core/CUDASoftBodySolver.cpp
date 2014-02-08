#include "CUDASoftBodySolver.h"
#include "common.h"
#include <cstring>

using namespace std;
using namespace glm;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


enum ArrayType {
    ARRAY_POSITIONS,
    ARRAY_PROJECTIONS,
    ARRAY_VELOCITIES,
    ARRAY_FORCES,
    ARRAY_LAST_DEFINED
};


struct CUDASoftBodySolver::SoftBodyDescriptor {
    SoftBody *body;
    cudaGraphicsResource *graphics;
    bool mapped : 1;
    unsigned int vertexBaseIdx;
    unsigned int linksBaseIdx;
    unsigned int mappingBaseIdx;
};

struct CUDASoftBodySolver::SolverPrivate {
    int             deviceId;
    cudaDeviceProp  devProp;
    cudaStream_t    stream;
    vec3            *array[ARRAY_LAST_DEFINED];
    uvec2           *links;
    float_t         *linksRestLength2;
    float_t         *massInv;
    uint_t          *mapping;  /* For updates only. keeps ids of particle per vertexes in VertexBuffer */
};

CUDASoftBodySolver::CUDASoftBodySolver(void)
    : mInitialized(false)
{
}

CUDASoftBodySolver::~CUDASoftBodySolver(void)
{
    shutdown();
}

bool CUDASoftBodySolver::initializeDevice(SolverPrivate *cuda)
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

bool CUDASoftBodySolver::shutdownDevice(SolverPrivate *cuda)
{
    cudaError_t err;

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) return false;

    err = cudaDeviceReset();
    if (err != cudaSuccess) return false;

    return true;
}

bool CUDASoftBodySolver::allocateDeviceBuffers(softbodyArray_t *bodies, SolverPrivate *cuda)
{
    int bytes2alloc = 0;
    long total_alloc = 0;
    cudaError_t err;

    FOREACH(it, bodies)
        bytes2alloc += (*it)->mParticles.size() * sizeof(vec3);

    for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; ++type) {
        err = cudaMalloc(&cuda->array[type], bytes2alloc);
        total_alloc += bytes2alloc;
        if (err != cudaSuccess) goto on_fail;
    }

    err = cudaMalloc(&cuda->massInv, bytes2alloc);
    if (err != cudaSuccess) goto on_fail;
    total_alloc += bytes2alloc;

    cudaMemset(cuda->array[ARRAY_FORCES], 0x0, bytes2alloc);

    bytes2alloc = 0;
    FOREACH(it, bodies)
        bytes2alloc += (*it)->mMeshVertexParticleMapping.size() * sizeof(uint_t);

    err = cudaMalloc(&cuda->mapping, bytes2alloc);
    if (err != cudaSuccess) goto on_fail;
    total_alloc += bytes2alloc;

    bytes2alloc = 0;
    FOREACH(it, bodies)
        bytes2alloc += (*it)->mLinks.size() * sizeof(uvec2);

    err = cudaMalloc(&cuda->links, bytes2alloc);
    if (err != cudaSuccess) goto on_fail;
    total_alloc += bytes2alloc;

    FOREACH(it, bodies)
        bytes2alloc += (*it)->mLinks.size() * sizeof(float_t);

    cudaMalloc(&cuda->linksRestLength2, bytes2alloc);
    if (err != cudaSuccess) goto on_fail;
    total_alloc += bytes2alloc;

    DBG("Device allocated mem: %ld bytes", total_alloc);

    return true;

on_fail:
    deallocateDeviceBuffers(cuda);
    return false;
}

void CUDASoftBodySolver::deallocateDeviceBuffers(SolverPrivate *cuda)
{
    for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; ++type)
        if (cuda->array[type]) cudaFree(cuda->array[type]);
    if (cuda->massInv) cudaFree(cuda->massInv);
    if (cuda->links) cudaFree(cuda->links);
    if (cuda->linksRestLength2) cudaFree(cuda->linksRestLength2);
}

bool CUDASoftBodySolver::copyBodiesToDeviceBuffers(softbodyArray_t *bodies, SolverPrivate *cuda)
{
    cudaError_t err;
    int idx = 0, idx2= 0;

    FOREACH(it, bodies) {
        SoftBodyDescriptor descr;
        SoftBody *body = *it;
        descr.body = *it;
        descr.mapped = false;
        descr.graphics = NULL;
        descr.vertexBaseIdx = idx;
        descr.linksBaseIdx = idx2;

        idx += body->mParticles.size();
        idx2 += body->mLinks.size();

        unsigned int bytes1 = body->mParticles.size() * sizeof(vec3);
        unsigned int bytes2 = body->mLinks.size() * sizeof(uvec2);
        unsigned int bytes3 = body->mLinks.size() * sizeof(float_t);

        unsigned int offset = idx * sizeof(vec3);
        unsigned int offset2 = idx2 * sizeof(uvec2);
        unsigned int offset3 = idx2 * sizeof(float_t);

        err = cudaMemcpy(cuda->array[ARRAY_POSITIONS] + offset, &(body->mParticles[0]),
                         bytes1, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) return false;

        err = cudaMemcpy(cuda->array[ARRAY_PROJECTIONS] + offset, &(body->mParticles[0]),
                         bytes1, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) return false;

        err = cudaMemcpy(cuda->array[ARRAY_VELOCITIES] + offset, &(body->mVelocities[0]),
                         bytes1, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) return false;

        err = cudaMemcpy(cuda->array[ARRAY_FORCES] + offset, &(body->mForces[0]), bytes1,
                         cudaMemcpyHostToDevice);
        if (err != cudaSuccess) return false;

        err = cudaMemset(cuda->massInv + offset3, body->mMassInv, bytes1);
        if (err != cudaSuccess) return false;

        vector<uvec2> tmp(body->mLinks.size());
        vector<float_t> tmp2(body->mLinks.size());

        FOREACH_R(lnk, body->mLinks)
        {
            tmp.push_back(lnk->index);
            tmp2.push_back(lnk->restLength);
        }

        err = cudaMemcpy(cuda->links + offset2, &tmp[0], bytes2, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) return false;

        err = cudaMemcpy(cuda->linksRestLength2 + offset3, &tmp2[0], bytes3, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) return false;

        if (body->mMesh) {
            const VertexBuffer *buf = body->mMesh->vertexes;
            if (buf) {
                switch(buf->getType()) {
                    case VertexBuffer::OPENGL_BUFFER:
                        if (!initGLGraphicsResource(static_cast<const GLVertexBuffer*>(buf), &descr))
                            return false;
                        break;
                    default:
                        break;
                }
            }
        }

        mDescriptors.push_back(descr);
    }

    return true;
}

bool CUDASoftBodySolver::initGLGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr)
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

bool CUDASoftBodySolver::initialize(softbodyArray_t *bodies)
{
    SolverPrivate *cuda;
    if (mInitialized) return true;

    cuda = new SolverPrivate;
    memset(cuda, 0x0, sizeof(SolverPrivate));

    if (!initializeDevice(cuda)) {
        ERR("CUDA Device initialization failed!");
        delete cuda;
        return false;
    }
    if (!allocateDeviceBuffers(bodies, cuda)) {
        shutdownDevice(mCuda);
        ERR("Unable to allocte enough memory on device!");
        delete cuda;
        return false;
    }

    if (!copyBodiesToDeviceBuffers(bodies, cuda)) {
        ERR("Error occured while copying Soft bodies data to device!");
        shutdownDevice(mCuda);
        deallocateDeviceBuffers(cuda);
        delete cuda;
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
        shutdownDevice(mCuda);
        deallocateDeviceBuffers(mCuda);
        delete mCuda;
        mCuda = NULL;
    }
    mInitialized = false;
}

bool CUDASoftBodySolver::updateVertexBuffers(void)
{
    return false;
}

void CUDASoftBodySolver::updateAllVertexBuffersAsync(void)
{
}

void CUDASoftBodySolver::projectSystem(float_t dt)
{
}
