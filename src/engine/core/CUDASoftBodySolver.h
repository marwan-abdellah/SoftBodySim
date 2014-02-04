#ifndef __CUDA_SOLVER_H
#define __CUDA_SOLVER_H

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "SoftBody.h"

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
		unsigned int vertex_base_idx;
		unsigned int links_base_idx;
	};

	public:
		CUDASoftBodySolver(void);
		~CUDASoftBodySolver(void);

		void 	initialize(std::vector<SoftBody>&);
		void	terminate(void);

		void 	projectSystem(glm::float_t dt);
		void 	solveCollisions(glm::float_t dt);
		void 	solveLinks(glm::float_t dt);
		void	integrateSystem(glm::float_t dt);

	private:
		bool 	initializeDevice(void);
		bool	serializeBodies(std::vector<SoftBody>&);
		void	shutdownDevice(void);

		cudaStream_t	m_stream;
		bool			m_initialized;
		std::vector<SoftBodyDescriptor> m_descriptors;
		glm::vec3		*m_array[ARRAY_LAST_DEFINED];
		glm::uvec2 		*m_links;
		glm::float_t	*m_links_rest_length2;
		glm::float_t	*m_mass_inv;
};

#endif
