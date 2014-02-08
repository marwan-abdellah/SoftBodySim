#ifndef __CUDA_SOLVER_H
#define __CUDA_SOLVER_H

#include "SoftBody.h"
#include "VertexBuffer.h"

#include <map>

typedef std::vector<SoftBody*>	 softbodyArray_t;

class CUDASoftBodySolver {
	public:

		CUDASoftBodySolver(void);
		~CUDASoftBodySolver(void);

		void 	initialize(softbodyArray_t *bodies);
		void	shutdown(void);

		void 	projectSystem(glm::float_t dt);

		bool	updateVertexBuffers(void);
		void	updateAllVertexBuffersAsync(void);

	private:
	    struct SolverPrivate;
        struct SoftBodyDescriptor;

        enum ArrayType {
            ARRAY_POSITIONS,
            ARRAY_PROJECTIONS,
            ARRAY_VELOCITIES,
            ARRAY_FORCES,
            ARRAY_LAST_DEFINED
        };

		void 	solveCollisions(glm::float_t dt);
		void 	solveLinks(glm::float_t dt);
		void	integrateSystem(glm::float_t dt);
		bool 	initializeDevice(SolverPrivate*);
		bool	copyBodiesToDevice(softbodyArray_t *bodies);
		void	freeBodies(void);
		bool    shutdownDevice(SolverPrivate*);

		void initGLGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr);

        SolverPrivate   *mCuda;

		bool			mInitialized;
		glm::vec3		*mArray[ARRAY_LAST_DEFINED];
		glm::uvec2 		*mLinks;
		glm::float_t	*mLinksRestLength2;
		glm::float_t	*mMassInv;

        std::vector<SoftBodyDescriptor>  mDescriptors;
};

#endif

