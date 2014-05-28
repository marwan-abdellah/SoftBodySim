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

enum CellIDType {
	CELL_HOME = 0,
	CELL_PHANTOM = 1,
	CELL_INVALID = 2
};

struct CellID {
	glm::uint_t objectID;
	glm::uint_t triangleID;
	glm::uint_t hash;
	bool control : 2;
};

struct SoftBodyDescriptor {
	SoftBody                  *body;
	cudaGraphicsResource      *graphics;
	glm::vec3                 *positions;
	glm::vec3                 *projections;
	glm::vec3                 *velocities;
	glm::vec3                 *forces;
	glm::float_t              *massesInv;
	unsigned int              nParticles;
	LinkConstraint            *links;
	unsigned int              nLinks;
	glm::uint_t               *mapping;  /* Mapping between particles positions and vertexes 
									   is VertexBuffer.
									 Used for updating Vertex poistions */
	unsigned int              nMapping;
	glm::uvec3                *triangles;
	int                       nTriangles;
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

__global__ void solveLinksConstraints(
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

__global__ void solveCollisionConstraints(
		glm::vec3 *projections,
		glm::float_t *masses_inv,
		glm::float_t ground_level,
		glm::uint_t max_idx);

__global__ void calculateSpatialHash(
		glm::uint_t objectID,
		glm::vec3 *projections,
		glm::float_t cellSize,
		CellID *cellIds,
		glm::uint_t max_idx);
