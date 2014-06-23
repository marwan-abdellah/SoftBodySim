#ifndef SBS_CUDA_KERNEL_H_
#define SBS_CUDA_KERNEL_H_

#include <glm/fwd.hpp>
#include "sbs/model/SoftBody.h"

#define MAX_LINKS 128

struct SoftBodyDescriptor {
	SoftBody                  *body;
	cudaGraphicsResource      *graphics;
	unsigned int              baseIdx;
	unsigned int              nParticles;
	unsigned int              linkIdx;
	unsigned int              nLinks;
	unsigned int              mappingIdx;
	unsigned int              nMapping;
	unsigned int              trianglesIdx;
	int                       nTriangles;
	int                       shapeDescr;
};

__global__ void cudaProjectPositionsAndVelocitiesKernel(
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
		glm::uint baseIdx,
		glm::uint mappingBaseIdx,
		glm::uint max_idx);

__global__ void solveLinksConstraints(
		unsigned int steps,
		LinkConstraint *links,
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::uint_t baseIdx,
		glm::uint_t linkIdx,
		glm::uint_t max_idx);

__global__ void calculateLinkStiffness(
		unsigned int solver_steps,
		LinkConstraint *links,
		glm::uint_t linkIdx,
		glm::uint_t max_idx);

__global__ void solveGroundWallCollisionConstraints(
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::float_t ground_level,
		glm::float_t left_wall,
		glm::float_t right_wall,
		glm::float_t front_wall,
		glm::float_t back_wall,
		glm::uint_t max_idx);

#endif
