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

        void    solveCollisions(glm::float_t dt);
        void    solveLinks(glm::float_t dt);
        void    integrateSystem(glm::float_t dt);

        bool    initializeDevice(SolverPrivate*);
        bool    shutdownDevice(SolverPrivate*);

        bool    allocateDeviceBuffers(softbodyArray_t *bodies, SolverPrivate*);
        void    deallocateDeviceBuffers(SolverPrivate *cuda);
        bool    copyBodiesToDeviceBuffers(softbodyArray_t *bodies, SolverPrivate*);

        bool initGLGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr);

        SolverPrivate   *mCuda;

        bool            mInitialized;
        std::vector<SoftBodyDescriptor>  mDescriptors;
};

#endif

