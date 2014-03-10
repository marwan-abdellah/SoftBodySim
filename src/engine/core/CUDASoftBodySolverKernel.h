#include "glm/glm.hpp"
#include "SoftBody.h"

#define MAX_LINKS 128

__global__ void cudaUpdateVelocitiesKernel(
	glm::vec3 gravity,
	glm::vec3 *positions,
	glm::vec3 *projections,
	glm::vec3 *velocities,
	glm::vec3 *ext_forces,
	glm::float_t *masses,
	glm::float_t dt,
	glm::uint_t max_idx);

__global__ void integrateMotionKernel(
		glm::float_t dt,
		glm::vec3 *positions,
		glm::vec3 *projections,
		glm::vec3 *velocities,
		glm::uint max_idx
		);

__global__ void cudaUpdateVertexBufferKernel(
		glm::vec3 *vboPtr,
		glm::vec3 *positions,
		glm::uint *mapping,
		glm::uint max_idx);

__global__ void solveConstraints(
		unsigned int steps,
		LinkConstraint *links,
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::uint_t max_idx);

