#ifndef SBS_SOLVER_CUDA_SOLVER_H_
#define SBS_SOLVER_CUDA_SOLVER_H_

#include "sbs/solver/SoftBodySolver.h"

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
