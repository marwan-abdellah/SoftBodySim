#include "glm/glm.hpp"
#include "CUDASoftBodySolverKernel.h"
#include "common.h"

using namespace glm;

__device__ uint_t hash(uint_t id)
{
	return 1193 * id;
}

__global__ void cudaUpdateVelocitiesKernel(
		vec3 gravity,
		vec3 *positions,
		vec3 *projections,
		vec3 *velocities,
		vec3 *ext_forces,
		float_t *masses,
		float_t dt,
		uint_t max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if ( idx < max_idx) {

		// 0. Load from global mem.
		vec3 position = positions[idx];
		vec3 force = ext_forces[idx];
		float_t imass = masses[idx];
		vec3 velocity = velocities[idx];
		
		// 1. Updating velocities.
		velocity += dt * imass * (force + gravity);

		// 2. Damp velocities.
		velocity *= 0.99f; // Naive damping

		// 3. projecting positions
		vec3 projection = position + velocity * dt;

		// update global tables
		projections[idx] = projection;
		velocities[idx] = velocity;
	}
}


/**
  step 4. solving links constraints.
  */
__global__ void solveConstraints(
		unsigned int max_steps,
		LinkConstraint *links,
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::uint_t max_idx)
{
	__shared__ vec3   ACCUM[2 * MAX_LINKS];
	__shared__ uint_t COUNTER[2 * MAX_LINKS];

	int link_idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (link_idx < max_idx) {

		LinkConstraint lnk = links[link_idx];
		glm::vec3 pos0 = projections[lnk.index[0]];
		glm::vec3 pos1 = projections[lnk.index[1]];
		glm::float_t mass_inv0 = masses_inv[lnk.index[0]];
		glm::float_t mass_inv1 = masses_inv[lnk.index[1]];

		// assume that will be no colliosions; MAX_LINS = 2^x, X in N
		uint_t id0 = hash(lnk.index[0]) & (2 * MAX_LINKS - 1);
		uint_t id1 = hash(lnk.index[1]) & (2 * MAX_LINKS - 1);

		ACCUM[id0] = pos0;
		ACCUM[id1] = pos1;
		COUNTER[id0] = 1;
		COUNTER[id1] = 1;

		__syncthreads();

		glm::float_t restLen = lnk.restLength;
		glm::float_t k = lnk.stiffness;

		glm::vec3 diff = pos0 - pos1;
		glm::float_t len = length(diff);

		float_t m0 = mass_inv0 / (mass_inv0 + mass_inv1) * (len - restLen) /
			len;
		float_t m1 = mass_inv1 / (mass_inv0 + mass_inv1) * (len - restLen) /
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

		pos0 = ACCUM[id0] * (1.0f / (float_t)COUNTER[id0]);
		pos1 = ACCUM[id1] * (1.0f / (float_t)COUNTER[id1]);

		projections[lnk.index[0]] = pos0;
		projections[lnk.index[1]] = pos1;
	}
}

//struct {
//	enum Type {
//		TRIANGLE,
//	} type;
//	union {
//		struct {
//			glm::uvec3 idx;
//		} triangle;
//	} data;
//	bool fixed;
//} CollisionBodyData;
//
///**
//  step 5. solving collision constraints.
//  */
//__global__ void solveCollisions(
//		glm::uvec2 *collisions,
//		CollisionBody *collisions_bodies_data,
//		glm::vec3 *projections,
//		glm::vec3 *positions,
//		glm::vec3 *velocities,
//		glm::vec3 *masses_inv,
//		glm::vec3 *forces,
//		glm::uint max_idx
//	)
//{
//	int coll_idx = blockIdx.x * blockDim.x + threadIdx.x;
//
//	if (coll_idx < max_idx) {
//		uvec2 *coll_data = collisions[coll_idx];
//
//		/**
//		  * coll_data[0] keeps id of colliding particle
//		  * coll_data[1] keeps id of colliding body
//		  */
//		glm::vec3 position = positions[coll_data[0]];
//		glm::vec3 projection = positions[coll_data[0]];
//		glm::float_t mass = masses_inv[coll_data[0]];
//		glm::vec3 force = forces[coll_data[0]];
//		glm::vec3 velocity = velocities[coll_data[0]];
//
//		CollisionBodyData body = collisions_bodies_data[coll_data[1]];
//
//		if (body.type == CollisionBody.TRIANGLE) {
//			glm::vec3 tri0 = positions[body.data.triangle.idx[0]];
//			glm::vec3 tri1 = positions[body.data.triangle.idx[1]];
//			glm::vec3 tri2 = positions[body.data.triangle.idx[2]];
//
//			// for barycentric coord test
//			glm::vec3 a = tri1 - tri0;
//			glm::vec3 b = tri2 - tri0;
//
//			glm::vec3 norm = glm::cross(a, b);
//			glm::vec3 diff = projection - position;
//			glm::float_t k = dot(norm, diff);
//			if (k < 0.0001f)
//				return;
//
//			k = dot(norm, (tri1 - position)) / k;
//			projection = position + k * diff;
//
//			// barycentric coord test
//			glm::vec3 c = projection - tri0;
//
//			glm::float_t dot00 = dot(a, a);
//			glm::float_t dot01 = dot(a, b);
//			glm::float_t dot02 = dot(a, c);
//			glm::float_t dot11 = dot(b, b);
//			glm::float_t dot12 = dot(b, c);
//
//			glm::float_t den = 1 / (dot00 * dot11 - dot01 * dot01);
//			glm::float_t u = (dot11 * dot02 - dot01 * dot12) * den;
//			glm::float_t v = (dot00 * dot12 - dot01 * dot02) * den;
//
//			if (u < 0 || v < 0 || u + v >= 1)
//				return;
//		}
//
//		projections[coll_data[0]] = projection;
//		velocities[coll_data[0]] = velocity;
//		forces[coll_data[0]] = force;
//	}
//}
//
///**
//  */
//
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

__global__ void cudaUpdateVertexBufferKernel(glm::vec3 *vboPtr, glm::vec3 *positions, glm::uint *mapping, glm::uint max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::uint index = mapping[idx];
		glm::vec3 vertex = positions[index];
		vboPtr[idx] = vertex;
	}
}

__global__ void calculateLinkStiffness(
		unsigned int solver_steps,
		LinkConstraint *links,
		glm::uint_t max_idx)
{
	int link_idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (link_idx < max_idx) {
		LinkConstraint lnk = links[link_idx];

		links[link_idx].stiffness = 1.0f - powf(1.0 - lnk.stiffness, 1.0f /
				solver_steps);
	}
}
