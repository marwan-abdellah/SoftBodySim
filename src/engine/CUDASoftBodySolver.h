#ifndef CUDA_SOLVER_H
#define CUDA_SOLVER_H

#include "SoftBody.h"
#include "VertexBuffer.h"

#include <list>

typedef std::list<SoftBody*>	 softbodyList_t; 
class CUDAContext;

class CUDASoftBodySolver {
	public:
		/**
		 * Default constructor
		 */
		CUDASoftBodySolver(void);

		/**
		 * Default destructor
		 */
		~CUDASoftBodySolver(void);

		/**
		 * Simulation parameters structure
		 *
		 * gravity - gravity vector (can be non-
		 */
		struct SoftBodyWorldParameters {
			glm::vec3   gravity;
			float_t     groundLevel;
		};

		bool initialize(void);
		void shutdown(void);

		void setWorldParameters(SoftBodyWorldParameters &params);

		void addSoftBodies(softbodyList_t &bodies);
		void removeBodies(softbodyList_t *bodies);
		void addSoftBody(SoftBody *body); void removeSoftBody(SoftBody *body);

		void projectSystem(glm::float_t dt);

		void updateVertexBuffers(void);
		void updateVertexBuffersAsync(void);

	private:
		CUDAContext *mContext;
		softbodyList_t   mBodies;
		bool			 mInitialized;
		SoftBodyWorldParameters mWorldParams;
};

#endif
