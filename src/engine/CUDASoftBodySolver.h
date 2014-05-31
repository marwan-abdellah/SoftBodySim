#ifndef CUDA_SOLVER_H
#define CUDA_SOLVER_H

#include "engine/solver/SoftBodySolver.h"


class CUDAContext;

class CUDASoftBodySolver : public SoftBodySolver {
	public:
		/**
		 * Default constructor
		 */
		CUDASoftBodySolver(void);

		/**
		 * Default destructor
		 */
		~CUDASoftBodySolver(void);

		bool Initialize(void);
		void Shutdown(void);
		void ProjectSystem(glm::float_t dt);
		void UpdateVertexBuffers(void);

		void AddSoftBody(SoftBody *body);
	private:
		CUDAContext *mContext;
		bool         mInitialized;
};

#endif
