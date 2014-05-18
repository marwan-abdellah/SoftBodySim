#include "glm/glm.hpp"
#include "SoftBody.h"

#define MAX_LINKS 128

struct CollisionPointTriangleConstraint2 {
	glm::vec3   *positions;
	glm::vec3   *projections;
	glm::uint_t pointIdx;
	glm::uvec3  triangleIdxs;
	glm::vec3   *trianglePositions;
	glm::vec3   *triangelProjections;
};

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
		MeshData::Vertex *vboPtr,
		glm::vec3 *positions,
		glm::uint *mapping,
		glm::uint max_idx);

__global__ void solveConstraints(
		unsigned int steps,
		LinkConstraint *links,
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::uint_t max_idx);

__global__ void calculateLinkStiffness(
		unsigned int solver_steps,
		LinkConstraint *links,
		glm::uint_t max_idx);

__global__ void solvePointTriangleCollisionsKernel(
		CollisionPointTriangleConstraint2 *collisions_data,
		glm::uint_t max_idx);
