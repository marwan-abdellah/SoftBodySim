#ifndef __CUDA_SOLVER_H
#define __CUDA_SOLVER_H

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "SoftBody.h"
#include "VertexBuffer.h"

#include <map>

struct SoftBodyDescriptor {
	SoftBody	*body;
	cudaGraphicsResource *graphics;
	bool mapped : 1;
	unsigned int vertexBaseIdx;
	unsigned int linksBaseIdx;
};

typedef std::map<const SoftBody*, SoftBodyDescriptor>	 descriptorMap_t;
typedef std::vector<SoftBodyDescriptor*>				 descriptorArray_t;
typedef std::vector<SoftBody*>							 softbodyArray_t;
typedef std::vector<VertexBuffer*>						 vertexBufferArray_t;
typedef std::vector<cudaGraphicsResource*>				 graphicsResourceArray_t;

class CUDASoftBodySolver {
	enum ArrayType {
		ARRAY_POSITIONS,
		ARRAY_PROJECTIONS,
		ARRAY_VELOCITIES,
		ARRAY_FORCES,
		ARRAY_LAST_DEFINED
	};

	public:

		CUDASoftBodySolver(void);
		~CUDASoftBodySolver(void);

		void 	initialize(softbodyArray_t *bodies);
		void	terminate(void);
		void 	projectSystem(glm::float_t dt);

		bool	updateVertexBuffers(void);
		void	updateAllVertexBuffersAsync(void);

	private:
		void 	solveCollisions(glm::float_t dt);
		void 	solveLinks(glm::float_t dt);
		void	integrateSystem(glm::float_t dt);

		bool 	initializeDevice(void);
		bool	copyBodiesToDevice(softbodyArray_t *bodies);
		void	freeBodies(void);
		void	shutdownDevice(void);

		void initGraphicsResource(const GLVertexBuffer *vb, SoftBodyDescriptor *descr);

		cudaStream_t	mStream;
		int				mDevId;
		bool			mInitialized;
		glm::vec3		*mArray[ARRAY_LAST_DEFINED];
		glm::uvec2 		*mLinks;
		glm::float_t	*mLinksRestLength2;
		glm::float_t	*mMassInv;

		descriptorMap_t	mDescriptorMap;
};

#endif

