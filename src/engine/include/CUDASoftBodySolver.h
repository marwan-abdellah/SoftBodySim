#ifndef __CUDA_SOLVER_H
#define __CUDA_SOLVER_H

#include "SoftBody.h"
#include "VertexBuffer.h"

#include <map>

typedef std::vector<SoftBody*>     softbodyArray_t;

class CUDASoftBodySolver {
    public:

        CUDASoftBodySolver(void);
        ~CUDASoftBodySolver(void);

        bool    initialize(softbodyArray_t *bodies);
        void    shutdown(void);

        void     projectSystem(glm::float_t dt);

        bool    updateVertexBuffers(void);
        void    updateAllVertexBuffersAsync(void);

    private:
        struct SolverPrivate;
        struct SoftBodyDescriptor;

        typedef std::vector<SoftBodyDescriptor> descriptorArray_t;

        void    solveCollisions(glm::float_t dt);
        void    solveLinks(glm::float_t dt);
        void    integrateSystem(glm::float_t dt);

        bool    initializeDevice(SolverPrivate*);
        bool    shutdownDevice(SolverPrivate*);

        void    createDescriptors(softbodyArray_t *bodies, descriptorArray_t *descriptors);
        bool    allocateDeviceBuffers(descriptorArray_t *, SolverPrivate*);
        void    deallocateDeviceBuffers(SolverPrivate*);
        bool    copyBodyToDeviceBuffers(SoftBodyDescriptor*, SolverPrivate*);

        bool registerVertexBuffers(SoftBodyDescriptor *descr);
        bool registerGLGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr);

        SolverPrivate   *mCuda;

        bool              mInitialized;
        descriptorArray_t mDescriptors;
};


#endif

