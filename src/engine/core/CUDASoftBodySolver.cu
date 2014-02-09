#include "glm/glm.hpp"
#include "CUDASoftBodySolver.h"
#include "common.h"

using namespace glm;

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
		velocity += dt * (force + gravity);

		// 2. Damp velocities.
		velocity *= 0.99f; // Naive damping

		// 3. projecting positions
		vec3 projection = position + velocity * dt;

		// update global tables
		projections[idx] = projection;
		positions[idx] = projection;
		velocities[idx] = velocity;
	}
}

//
///**
//  step 4. solving links constraints.
//  */
//__global__ void solveLinks(
//		glm::float_t k,
//		glm::uvec2 *links,
//		glm::vec3 *projections,
//		glm::float_t *masses_inv,
//		glm::float_t *rest_length2,
//		glm::uint_t max_idx)
//{
//	int link_idx = blockIdx.x * blockDim.x + threadIdx.x;
//
//	if (link_idx < max_idx) {
//		glm::float_t restLen2 = rest_length2[link_idx];
//		glm::uvec2 idx = links[link_idx];
//
//		glm::vec3 pos0 = projections[idx[0]];
//		glm::vec3 pos1 = projections[idx[1]];
//
//		glm::float_t mass_inv0 = masses_inv[idx[0]];
//		glm::float_t mass_inv1 = masses_inv[idx[1]];
//
//		glm::vec3 dist = pos0 - pos1;
//		glm::float_t len2 = glm::dot(dist, dist);
//		glm::float_t c = k * (restLen2 - len2);
//
//		pos0 = pos0 - c * mass_inv0 * dist;
//		pos1 = pos1 + c * mass_inv1 * dist;
//
//		projections[idx[0]] = pos0;
//		projections[idx[1]] = pos1;
//	}
//}
//
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
///**
//  step 6. Integrate motion.
//  */
//__global__ void integrateMotion(
//		glm::float_t dt,
//		glm::vec3 *positions,
//		glm::vec3 *projections,
//		glm::vec3 *velocities,
//		glm::uint max_idx
//		)
//{
//	int idx = blockIdx.x * blockDim.x + threadIdx.x;
//
//	if (idx < max_idx) {
//		glm::vec3 pos = positions[idx];
//		glm::vec3 proj = projections[idx];
//
//		velocities[idx] = (proj - pos) * dt;
//		positions[idx] = proj
//	}
//}

struct BufferMapping {
	glm::vec3 *vboPtr;
	unsigned int baseIdx;
};

__global__ void cudaUpdateVertexBufferKernel(BufferMapping mapp, glm::vec3 *positions, glm::uint *mapping, glm::uint max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::uint index = mapping[idx];
		glm::vec3 vertex = positions[index];
		mapp.vboPtr[idx - mapp.baseIdx] = vertex;
	}
}

void CUDASoftBodySolver::cudaUpdateVertexBuffer(glm::vec3 *positions, glm::uint
		*mapping, glm::vec3 *vboPtr, unsigned int baseIdx,
		unsigned int len)
{
	BufferMapping b;
	b.vboPtr = vboPtr;
	b.baseIdx = baseIdx;
	int threadCount = 128;
	int blockCount = len / threadCount + 1;
	cudaUpdateVertexBufferKernel<<<blockCount, threadCount >>>(b, positions, mapping, len);
}

void CUDASoftBodySolver::cudaProjectSystem(float_t dt, vec3 *gravity, vec3
		*positions, vec3 *velocities, vec3 *forces, vec3 *projections, float_t *massInv, uint_t
		maxId)
{
	int threadCount = 128;
	int blockCount = maxId / threadCount + 1;
	cudaUpdateVelocitiesKernel<<<blockCount, threadCount>>>(*gravity, positions,
			projections, velocities, forces, massInv, dt, maxId);
}
