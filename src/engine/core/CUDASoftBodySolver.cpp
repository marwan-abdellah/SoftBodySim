#include "CUDASoftBodySolver.h"
#include "common.h"

using namespace std;
using namespace glm;

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


struct CUDASoftBodySolver::SoftBodyDescriptor {
    SoftBody *body;
    cudaGraphicsResource *graphics;
    bool mapped : 1;
    unsigned int vertexBaseIdx;
    unsigned int linksBaseIdx;
};

struct CUDASoftBodySolver::SolverPrivate {
    int             deviceId;
    cudaDeviceProp  devProp;
    cudaStream_t    stream;
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

    // let runtime api choose device for us.
    err = cudaChooseDevice(&cuda->deviceId, NULL);
    if (err != cudaSuccess) return false;

    err = cudaSetDevice(cuda->deviceId);
    if (err != cudaSuccess) return false;

    err = cudaGetDeviceProperties(&mCuda->devProp, mCuda->deviceId);
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
        descr.mapped = false;
        descr.graphics = NULL;
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

        if (body->mMesh) {
            const VertexBuffer *buf = body->mMesh->vertexes;
            if (buf) {
                switch(buf->getType()) {
                    case VertexBuffer::OPENGL_BUFFER:
                        initGLGraphicsResource(static_cast<const GLVertexBuffer*>(buf), &descr);
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

void CUDASoftBodySolver::initGLGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr)
{
    cudaError_t err;
    GLuint id = vb->getVBO(GLVertexBuffer::VERTEX_ATTR_POSITION);
    err = cudaGraphicsGLRegisterBuffer(&descr->graphics, id, cudaGraphicsRegisterFlagsNone);
    if (err != cudaSuccess) {
        ERR("Unable to register GL buffer object %d", id);
        descr->graphics = NULL;
    }
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

    mCuda = new SolverPrivate;

    if (!initializeDevice(mCuda)) {
        ERR("CUDA Device initialization failed!");
        delete mCuda;
        return;
    }

    if (!copyBodiesToDevice(bodies)) {
        ERR("Unable to copy Soft bodies to device!");
        shutdownDevice(mCuda);
        freeBodies();
        return;
    }

    mInitialized = true;
}

void CUDASoftBodySolver::shutdown(void)
{
    if (!mInitialized) return;

    if (mCuda) {
        shutdownDevice(mCuda);
        delete mCuda;
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
