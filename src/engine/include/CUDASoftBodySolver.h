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

		bool initialize(softbodyArray_t *bodies);
		void shutdown(void);

		void projectSystem(glm::float_t dt);

		void updateVertexBuffers(void);
		void updateVertexBuffersAsync(void);

	private:
		struct SolverPrivate;
		struct SoftBodyDescriptor;
		struct CollisionBodyInfoDescriptor;

		typedef std::vector<SoftBodyDescriptor> descriptorArray_t;
		typedef std::vector<CollisionBodyInfoDescriptor> collisionBodyDescriptorArray_t;

		void updateVertexBuffers(glm::vec3*, unsigned int, unsigned int);

		void solveCollisions(glm::float_t dt);
		void solveLinks(glm::float_t dt);
		void integrateSystem(glm::float_t dt);

		SolverPrivate *cudaContextCreate(softbodyArray_t*);
		void cudaContextShutdown(SolverPrivate*);

		bool cudaInitializeDevice(SolverPrivate*);
		bool cudaShutdownDevice(SolverPrivate*);
		bool cudaInitCollisionDescriptors(SolverPrivate *cuda);

		SoftBodyDescriptor cudaCreateDescriptor(SoftBody *body);
		void cudaAppendCollsionDescriptors(collisionBodyDescriptorArray_t *, SoftBodyDescriptor *);
		long cudaAllocateDeviceBuffers(SoftBodyDescriptor*);
		void cudaDeallocateDeviceBuffers(SoftBodyDescriptor*);
		bool cudaCopyBodyToDeviceBuffers(SoftBodyDescriptor*);
		bool cudaRegisterVertexBuffers(SoftBodyDescriptor *descr);

		void updateVertexBuffers(SolverPrivate *cuda, bool async);
		void projectSystem(SolverPrivate *cuda, float_t dt);

		SolverPrivate   *mCuda;
		bool			 mInitialized;
		glm::vec3		 mGravity;
};


#endif

