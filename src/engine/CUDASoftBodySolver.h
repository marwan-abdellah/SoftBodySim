#ifndef CUDA_SOLVER_H
#define CUDA_SOLVER_H

#include "SoftBody.h"
#include "VertexBuffer.h"

#include <list>

typedef std::list<SoftBody*>	 softbodyList_t; 

class CUDASoftBodySolver {
	public:

		CUDASoftBodySolver(void);
		~CUDASoftBodySolver(void);

		struct SoftBodyWorldParameters {
			glm::vec3   gravity;
			float_t     groundLevel;
		};

		bool initialize(void);
		void shutdown(void);

		void setWorldParameters(SoftBodyWorldParameters &params);

		void addSoftBodies(softbodyList_t &bodies);
		void removeBodies(softbodyList_t *bodies);

		void addSoftBody(SoftBody *body);
		void removeSoftBody(SoftBody *body);

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

		SolverPrivate *cudaContextCreate(softbodyList_t*);
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
		void cudaUpdateConstraintStiffness(SoftBodyDescriptor *descr, int solverSteps);

		void updateVertexBuffers(SolverPrivate *cuda, bool async);
		void projectSystem(SolverPrivate *cuda, float_t dt);

		SolverPrivate   *mCuda;
		softbodyList_t   mBodies;
		bool			 mInitialized;
		SoftBodyWorldParameters mWorldParams;
};


#endif

