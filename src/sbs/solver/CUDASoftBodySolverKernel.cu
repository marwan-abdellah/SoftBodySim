#include "common.h"

#define GL_FORCE_CUDA
#include <glm/glm.hpp>
#include "sbs/solver/Math.h"
#include "sbs/solver/CUDASoftBodySolverKernel.h"

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

__global__ void solveShapeMatchingConstraints1(
		ParticleInfo *info_array,
		ShapeRegionStaticInfo *regions,
		glm::uint_t *members_offsets,
		glm::vec3 *shapes_init_positions,
		glm::vec3 *projections,
		glm::float_t *masses,
		ShapeRegionDynamicInfo *results,
		glm::uint_t max_idx
		)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::mat3 A(0), S;
		ParticleInfo info = info_array[idx];
		ShapeRegionStaticInfo reg = regions[info.region_id];
		ShapeRegionDynamicInfo res;
		res.R = glm::mat3(0);
		res.mc = glm::vec3(0,0,0);

		glm::uint_t *members = members_offsets +
			reg.members_offsets_offset;
		glm::vec3 *inits = shapes_init_positions +
			reg.shapes_init_positions_offset;

		for (int i = 0; i < reg.n_particles; ++i) {
			glm::uint mem_offset = members[i];
			glm::vec3 init = inits[mem_offset];

			glm::uint_t offset = info.body_offset + mem_offset;
			glm::vec3 proj = projections[offset];
			glm::float_t mass = masses[offset];
			A += mass * glm::outerProduct(proj, init);
			res.mc += proj * mass;
		}

		res.mc = res.mc / reg.mass;
		A -= reg.mass * glm::outerProduct(res.mc, reg.mc0);

		polar_decomposition(A, res.R, S);

		res.mc0 = reg.mc0;
		results[idx] = res;
	}
}

__global__ void solveShapeMatchingConstraints2(
		ParticleInfo *info_array,
		ShapeRegionStaticInfo *regions,
		ShapeRegionDynamicInfo *results,
		glm::uint_t *region_members,
		glm::vec3 *shapes_initial_positions,
		glm::uint_t *particles_regions,
		glm::vec3 *projections,
		glm::uint_t max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		ParticleInfo info = info_array[idx];
		ShapeRegionStaticInfo reg = regions[info.region_id];
		ShapeRegionDynamicInfo dinfo = results[idx];

		glm::vec3 proj = projections[idx];

		glm::uint_t *part_reg_start = particles_regions +
			reg.regions_offsets_offset;
		
		glm::vec3 init = shapes_initial_positions[idx];
		glm::vec3 final(0,0,0);
		glm::mat3 Rfinal(0);

		for (int i = 0; i < reg.n_regions; ++i) {
			glm::uint id = part_reg_start[i];
			ShapeRegionDynamicInfo res = results[id];
			final += res.R * (init - res.mc0) + res.mc;
		}
		final = final / (glm::float_t)reg.n_regions;

		proj += 0.9f * (final - proj);
		projections[idx] = proj;
	}
}

__global__ void solveVolumePreservationConstraint1(
		ParticleInfo *info,
		glm::float_t *partials,
		glm::vec3 *projections,
		glm::uvec3 *triangles,
		glm::vec3 *normals,
		glm::uint_t max_idx)
{
	__shared__ glm::float_t partial_volumes[128];

	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	partial_volumes[threadIdx.x] = 0;

	if (idx < max_idx) {
		glm::uvec3 indexes = triangles[idx];

		glm::vec3 v0 = projections[indexes[0]];
		glm::vec3 v1 = projections[indexes[1]];
		glm::vec3 v2 = projections[indexes[2]];

		glm::vec3 norm = glm::cross(v1 - v0, v2 - v0);
		if (norm != glm::vec3(0,0,0)) {
			norm = glm::normalize(norm);
		}

		glm::float_t area = triangle_area(v0, v1, v2);
		partial_volumes[threadIdx.x] = area * glm::dot(v0 + v1 + v2, norm);
		normals[idx] = norm;

		__syncthreads();

		for (unsigned int s = 1; s < blockDim.x; s = 2 * s) {
			if ((threadIdx.x % (2 * s)) == 0)
				partial_volumes[threadIdx.x] += partial_volumes[threadIdx.x + s];
			__syncthreads();
		}

		if (threadIdx.x == 0) partials[blockIdx.x] = partial_volumes[0] / 3.0f;
	}
}

__global__ void solveVolumePreservationConstraint2(
		SoftBodyDescriptor *descrs,
		glm::float_t *partials,
		glm::uint_t max_idx)
{
	__shared__ glm::float_t volumes[128];

	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	volumes[threadIdx.x] = 0;

	if (idx < max_idx) {
		volumes[threadIdx.x] = partials[idx];

		__syncthreads();

		for (unsigned int s = 1; s < blockDim.x; s = 2 * s) {
			if ((threadIdx.x % (2 * s)) == 0)
				volumes[threadIdx.x] += volumes[threadIdx.x + s];
			__syncthreads();
		}

		if (threadIdx.x == 0) descrs[0].volume = volumes[0];
	}
}

__global__ void solveVolumePreservationConstraint3(
		ParticleTrianglesInfo *infos,
		SoftBodyDescriptor *descrs,
		glm::vec3 *normals,
		glm::vec3 *projections,
		glm::uint_t *indexes,
		glm::uint_t max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		ParticleTrianglesInfo info = infos[idx];
		SoftBodyDescriptor descr = descrs[0];
		glm::vec3 normal(0,0,0);
		for (int i = 0; i < info.n_triangles; ++i) {
			glm::uint_t index = indexes[info.triangle_id_offset + i];
			normal += normals[index];
		}
		normal = normal / (glm::float_t)info.n_triangles;

		glm::vec3 proj = projections[idx];
		glm::float_t diff = (descr.volume - descr.volume0) / descr.volume0;
		if (diff > -0.05f) return;
		proj -= diff * 0.1f * normal;

		projections[idx] = proj;
	}
}

__global__ void solveGroundWallCollisionConstraints(
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::float_t ground_level,
		glm::float_t left_wall,
		glm::float_t right_wall,
		glm::float_t front_wall,
		glm::float_t back_wall,
		glm::uint_t max_idx)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < max_idx) {
		glm::vec3 pos = projections[idx];

		pos[1] = pos[1] < ground_level ? ground_level : pos[1];
		pos[0] = pos[0] < left_wall ? left_wall : pos[0];
		pos[0] = pos[0] > right_wall ? right_wall : pos[0];
		pos[2] = pos[2] > front_wall ? front_wall : pos[2];
		pos[2] = pos[2] < back_wall ? back_wall : pos[2];

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
