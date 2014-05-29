#include "glm/glm.hpp"
#include "CUDASoftBodySolverKernel.h"
#include "common.h"

using namespace glm;

__device__ uint_t hash(uint_t id)
{
	return 1193 * id;
}

__global__ void cudaProjectPositionsAndVelocitiesKernel(
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
__global__ void solveLinksConstraints(
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

__global__ void solveCollisionConstraints(
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
  step 5. solving collision constraints.
  */
__global__ void solvePointTriangleCollisionsKernel(
		SoftBodyDescriptor *descriptors,
		PointTriangleConstraint *collisions_data,
		uint_t max_idx
	)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		PointTriangleConstraint cd = collisions_data[idx];

		vec3 position = descriptors[cd.pointObjectId].positions[cd.pointIdx];
		vec3 projection = descriptors[cd.pointObjectId].projections[cd.pointIdx];

		uvec3 triIds = descriptors[cd.triangleObjectId].triangles[cd.triangleId];
		vec3 tri0 = descriptors[cd.triangleObjectId].projections[triIds[0]];
		vec3 tri1 = descriptors[cd.triangleObjectId].projections[triIds[1]];
		vec3 tri2 = descriptors[cd.triangleObjectId].projections[triIds[2]];

		vec3 e0 = tri2 - tri0;
		vec3 e1 = tri1 - tri0;

		// calculate triangle's plane normal
		vec3 norm = cross(e0, e1);

		// quick test to determing if projection and position
		// vertexes are on same side of triangle.
		//if (dot(projection, norm) * dot(position, norm) < 0.0f) return;

		// ray direction
		vec3 dir = projection - position;

		// estimate plane d coefficient
		float_t d = dot(norm, tri0);

		// calculate intersection point
		if (dot(norm, dir) == 0.00f) return;
		float_t q = (d - dot(norm, position)) / dot(norm, dir);
		vec3 Q = position + q * dir;

		// check if movement is long enough to intersect plane
		if (q > 1.0 || q < 0.0) return;

		// barycentric coordinates test
		vec3 e2 = Q - tri0;

		if (dot(e2, norm) > 0) return;

		float_t dot00 = dot(e0, e0);
		float_t dot01 = dot(e0, e1);
		float_t dot02 = dot(e0, e2);
		float_t dot11 = dot(e1, e1);
		float_t dot12 = dot(e1, e2);

		float_t den = 1.0 / (dot00 * dot11 - dot01 * dot01);
		float_t u = (dot11 * dot02 - dot01 * dot12) * den;
		float_t v = (dot00 * dot12 - dot01 * dot02) * den;

		if (u >= 0 && v >= 0 && (u + v) < 1.0) {
			Q += 0.02f * -norm;
			descriptors[cd.pointObjectId].projections[cd.pointIdx] = Q;
		}
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

__global__ void cudaUpdateVertexBufferKernel(MeshData::Vertex *vboPtr, glm::vec3 *positions, glm::uint *mapping, glm::uint max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::uint index = mapping[idx];
		glm::vec3 vertex = positions[index];
		vboPtr[idx].position = vertex;
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

__global__ void calculateSpatialHash(
		glm::uint_t objectID,
		glm::uint_t baseIdx,
		glm::uvec3 *triangles,
		glm::vec3 *projections,
		glm::float_t cellSize,
		CellID *cellIds,
		glm::uint_t max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	CellID cell;

	if (idx < max_idx) {
		uvec3 ids = triangles[idx];
		vec3 a = projections[ids[0]];
		vec3 b = projections[ids[1]];
		vec3 c = projections[ids[2]];
		vec3 mid = (a + b + c) / 3.0f;

		ids = uvec3(mid[0] / cellSize, mid[1] / cellSize, mid[2] / cellSize);

		cellIds[baseIdx + idx] = cell;
	}
}
