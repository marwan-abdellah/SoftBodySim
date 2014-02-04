#ifndef __CUDA_SOLVER_H
#define __CUDA_SOLVER_H

#include "SoftBody.h"

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


class CUDASoftBody : SoftBody {
	public:
		CUDASoftBody(SoftBody &body);
		~CuDASoftBody(SoftBody &body);

		SoftBody	*getSoftBody(void);

		SoftBody 	*m_body;
		int 		m_vertex_base_idx;
		int 		m_links_base_idx;
};

CUDASoftBody::CUDASoftBody(SoftBody &body) :
	m_body(&body)
{
}

SoftBody *CUDASoftBody::getSoftBody(void)
{
	return m_body;
}


class CUDASoftBodySolver {
	public:
		CUDASoftBodySolver(int dev_id);
		~CUDASoftBodySolver(void);

		void 	initialize(vector<SoftBody*>&, vector<CollisionBody*>&);
		void	terminate(void);

		void 	projectSystem(glm::float_t dt);
		void 	solveCollisions(glm::float_t dt);
		void 	solveLinks(glm::float_t dt);
		void	integrateSystem(glm::float_t dt);

		void	copyDataToVertexBuffer(SoftBody *sb, VertexBuffer&);

	private:
		cudaStream_t	*m_stream;
		bool			m_initialized;
		glm::vec3		*m_positions;
		glm::vec3		*m_projections;
		glm::vec3		*m_velocities;
		glm::vec3		*m_forces;
};

CUDASoftBodySolver::CUDASoftBodySolver(int dev_id)
	:
		m_initialized(false)
{
}

CUDASoftBodySolver::~CUDASoftBodySolver(void)
{
	if (m_initialized)
		terminate();
}

void CUDASoftBodySolver::initialize(vector<SoftBody*>&, vector<CollisionBody*>&)
{
	cudaError_t error;
	int device_count;
	unsigned int total_particles = 0;

	error = cudaGetDeviceCount(&device_count);
	if (error != cudaSuccess)
		SB_ASSERT("%s", cudaGetErrorString(error));

	if (dev_id < 0 || dev_id >= device_count)
		SB_ASSERT("Device ID less then 0, or greater then device count");

	error = cudaSetDevice(dev_id);
	if (error != cudaSuccess)
		SB_ASSERT("%s", cudaGetErrorString(error));
	
	error = cudaStreamCreate(&m_stream);
	if (error != cudaSuccess)
		SB_ASSERT("%s", cudaGetErrorString(error));

	for (

	m_initialized = true;
}

void CUDASoftBodySolver::terminate(void)
{
	cudaStreamDestroy(m_stream);
	m_initialized = false;
}

#endif
