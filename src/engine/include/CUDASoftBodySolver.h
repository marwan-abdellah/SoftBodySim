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

        void	updateVertexBuffers(void);
        void    updateVertexBuffersAsync(void);

        struct SolverPrivate;
        struct SoftBodyDescriptor;
    private:

		void   updateVertexBuffers(glm::vec3*, unsigned int, unsigned int);

        void    solveCollisions(glm::float_t dt);
        void    solveLinks(glm::float_t dt);
        void    integrateSystem(glm::float_t dt);

		SolverPrivate *cudaContextCreate(void);
        bool    cudaInitialize(SolverPrivate*, softbodyArray_t*);
        void 	cudaShutdown(SolverPrivate*);

        bool    cudaInitializeDevice(SolverPrivate*);
        bool    cudaShutdownDevice(SolverPrivate*);

        void    cudaCreateDescriptors(SolverPrivate*, softbodyArray_t *bodies);
        bool    cudaAllocateDeviceBuffers(SolverPrivate*);
		bool    cudaInitializeBodies(SolverPrivate *cuda);
        void    cudaDeallocateDeviceBuffers(SolverPrivate*);
        bool    cudaCopyBodyToDeviceBuffers(SoftBodyDescriptor*, SolverPrivate*);
		void    cudaUpdateVertexBuffer(glm::vec3 *positions, glm::uint
				*mapping, glm::vec3 *vboPtr, unsigned int baseIdx, unsigned int len);

        bool    cudaRegisterVertexBuffers(SoftBodyDescriptor *descr, SolverPrivate*);
        bool    cudaRegisterGLGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr);

		void    updateVertexBuffers(SolverPrivate *cuda, bool async);

        SolverPrivate   *mCuda;
        bool             mInitialized;
};


#endif

