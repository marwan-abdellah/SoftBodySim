#ifndef __CUDA_SOLVER_H
#define __CUDA_SOLVER_H

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "SoftBody.h"
#include "VertexBuffer.h"

class CUDASoftBodySolver {
	enum ArrayType {
		ARRAY_POSITIONS,
		ARRAY_PROJECTIONS,
		ARRAY_VELOCITIES,
		ARRAY_FORCES,
		ARRAY_LAST_DEFINED
	};

	struct SoftBodyDescriptor {
		SoftBody	*body;
		unsigned int vertexBaseIdx;
		unsigned int linksBaseIdx;
	};

	public:
		CUDASoftBodySolver(void);
		~CUDASoftBodySolver(void);

		void 	initialize(std::vector<SoftBody>&);
		void	terminate(void);
		void 	projectSystem(glm::float_t dt);

		void 	synchronizeBody(SoftBody *sb);
		void 	synchronizeAll(void);

		VertexBuffer	*copySBToVertexBuffer(SoftBody *sb);

	private:
		void 	solveCollisions(glm::float_t dt);
		void 	solveLinks(glm::float_t dt);
		void	integrateSystem(glm::float_t dt);

		bool 	initializeDevice(void);
		bool	copyBodiesToDevice(std::vector<SoftBody>&);
		void	freeBodies(void);
		void	shutdownDevice(void);

		cudaStream_t	mStream;
		int				mDevId;
		bool			mInitialized;
		std::vector<SoftBodyDescriptor> mDescriptors;
		glm::vec3		*mArray[ARRAY_LAST_DEFINED];
		glm::uvec2 		*mLinks;
		glm::float_t	*mLinksRestLength2;
		glm::float_t	*mMassInv;
};

#endif
