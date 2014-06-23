#include "common.h"

#include <glm/glm.hpp>
#include "sbs/solver/CUDASoftBodySolverKernel.h"

__device__ glm::uint_t hash(glm::uint_t id)
{
	return 1193 * id;
}

__global__ void cudaProjectPositionsAndVelocitiesKernel(
		glm::vec3 gravity,
		glm::vec3 *positions,
		glm::vec3 *projections,
		glm::vec3 *velocities,
		glm::vec3 *ext_forces,
		glm::float_t *masses,
		glm::float_t dt,
		glm::uint_t max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if ( idx < max_idx) {

		// 0. Load from global mem.
		glm::vec3 force(0,0,0);
		if (ext_forces)
			glm::vec3 force = ext_forces[idx];

		glm::vec3 position = positions[idx];
		glm::float_t imass = masses[idx];
		glm::vec3 velocity = velocities[idx];
		
		// 1. Updating velocities.
		velocity += dt * imass * (force + gravity);

		// 2. Damp velocities.
		velocity *= 0.99f; // Naive damping

		// 3. projecting positions
		glm::vec3 projection = position + velocity * dt;

		// update global tables
		projections[idx] = projection;
		velocities[idx] = velocity;
	}
}

/**
  step 4. solving links constraints.
  */
__global__ void solveLinksConstraints(
		unsigned int max_steps,
		LinkConstraint *links,
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::uint_t baseIdx,
		glm::uint_t linkIdx,
		glm::uint_t max_idx)
{
	__shared__ glm::vec3   ACCUM[2 * MAX_LINKS];
	__shared__ glm::uint_t COUNTER[2 * MAX_LINKS];

	int link_idx = blockIdx.x * blockDim.x + threadIdx.x + linkIdx;

	if (link_idx < max_idx) {

		LinkConstraint lnk = links[link_idx];
		glm::vec3 pos0 = projections[lnk.index[0] + baseIdx];
		glm::vec3 pos1 = projections[lnk.index[1] + baseIdx];
		glm::float_t mass_inv0 = masses_inv[lnk.index[0] + baseIdx];
		glm::float_t mass_inv1 = masses_inv[lnk.index[1] + baseIdx];

		// assume that will be no colliosions; MAX_LINS = 2^x, X in N
		glm::uint_t id0 = hash(lnk.index[0]) & (2 * MAX_LINKS - 1);
		glm::uint_t id1 = hash(lnk.index[1]) & (2 * MAX_LINKS - 1);

		ACCUM[id0] = pos0;
		ACCUM[id1] = pos1;
		COUNTER[id0] = 1;
		COUNTER[id1] = 1;

		__syncthreads();

		glm::float_t restLen = lnk.restLength;
		glm::float_t k = lnk.stiffness;

		glm::vec3 diff = pos0 - pos1;
		glm::float_t len = glm::length(diff);

		glm::float_t m0 = mass_inv0 / (mass_inv0 + mass_inv1) * (len - restLen) /
			len;
		glm::float_t m1 = mass_inv1 / (mass_inv0 + mass_inv1) * (len - restLen) /
			len;

		pos0 -= k * m0 * diff;
		pos1 += k * m1 * diff;

		atomicAdd(&ACCUM[id0][0], pos0[0]);
		atomicAdd(&ACCUM[id0][1], pos0[1]);
		atomicAdd(&ACCUM[id0][2], pos0[2]);
		atomicAdd(&ACCUM[id1][0], pos1[0]);
		atomicAdd(&ACCUM[id1][1], pos1[1]);
		atomicAdd(&ACCUM[id1][2], pos1[2]);

		atomicInc(&COUNTER[id0], MAX_LINKS);
		atomicInc(&COUNTER[id1], MAX_LINKS);

		__syncthreads();

		pos0 = ACCUM[id0] * (1.0f / (glm::float_t)COUNTER[id0]);
		pos1 = ACCUM[id1] * (1.0f / (glm::float_t)COUNTER[id1]);

		projections[lnk.index[0] + baseIdx] = pos0;
		projections[lnk.index[1] + baseIdx] = pos1;
	}
}

__global__ void solveGroundCollisionConstraints(
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::float_t ground_level,
		glm::uint_t max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::vec3 pos = projections[idx];

		if (pos[1] < ground_level)
			pos[1] = ground_level;

		projections[idx] = pos;
	}
}

/**
  step 6. Integrate motion.
  */
__global__ void integrateMotionKernel(
		glm::float_t dt,
		glm::vec3 *positions,
		glm::vec3 *projections,
		glm::vec3 *velocities,
		glm::uint max_idx
		)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::vec3 pos = positions[idx];
		glm::vec3 proj = projections[idx];

		velocities[idx] = (proj - pos) / dt;
		positions[idx] = proj;
	}
}

__global__ void cudaUpdateVertexBufferKernel(glm::vec3 *vboPtr, glm::vec3
		*positions, glm::uint *mapping, glm::uint_t baseIdx, glm::uint_t mappingBaseIdx,
		glm::uint max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::uint index = mapping[mappingBaseIdx + idx];
		glm::vec3 vertex = positions[baseIdx + index];
		vboPtr[idx] = vertex;
	}
}

__global__ void calculateLinkStiffness(
		unsigned int solver_steps,
		LinkConstraint *links,
		glm::uint_t linkBaseIdx,
		glm::uint_t max_idx)
{
	int link_idx = blockIdx.x * blockDim.x + threadIdx.x + linkBaseIdx;

	if (link_idx < max_idx) {
		LinkConstraint lnk = links[link_idx];

		links[link_idx].stiffness = 1.0f - powf(1.0 - lnk.stiffness, 1.0f /
				solver_steps);
	}
}
