#ifndef SOFTBODY_H
#define SOFTBODY_H

#include "geometry/Arrays.h"
#include "Body.h"
#include "model/MeshData.h"

class SoftBody;
class SoftBodySolver;
class CUDASoftBodySolver;
class CPUSoftBodySolver;
class CUDACOntext;

/**
 * @struct LinkConstraint
 * @brief Data type describing connection between two mass points in SoftBody model.
 * When connection prelongs or shortens 
 *
 * @var LinkConstraint::index
 * indexes  of connected particles
 * @var LinkConstraint::restLength
 * initial, rest length of connection.
 * @var LinkConstraint::stiffness 
 * parameter describing strength of connection. Must be in [0, 1].
 */
struct LinkConstraint {
    glm::uvec2          index;
    glm::float_t        restLength;
	glm::float_t		stiffness;
};

/**
 * @struct VolumeConstraint
 * @brief Data type describing volume preservation tetrahedra.
 *
 * @var VolumeConstraint::index
 * @var VolumeConstraint::restLength
 */
struct VolumeConstraint {
    glm::uvec4          index;
    glm::float_t        restVolume;
};

/**
 * @struct CollisionConstraint
 * @brief Point-Triangle collision description.
 *
 * @var CollisionConstraint::pointIdx
 * index of colliding point
 * @var CollisionConstraint::triangleIdxs
 * indexes of colliding triangle
 */
struct CollisionConstraint {
	glm::uint_t         pointIdx;
	glm::uvec3          triangleIdxs;
};

typedef std::vector<LinkConstraint>             linksArray_t;
typedef std::vector<VolumeConstraint>           volumeArray_t;
typedef std::vector<CollisionConstraint>        collisionArray_t;

class SoftBody : public Body {
public:
    SoftBody(glm::float_t mass, glm::float_t springness,
             MeshData *mesh);
    ~SoftBody(void) {}

	float_t GetSpringness(void) { return mSpringiness; }
	void SetSpringness(glm::float_t val) { mSpringiness = val; }

	/**
	 * @brief Sets an distance constraint between particles.
	 *
	 * @param[in] array of LinkConstraints
	 */
	void SetLinkConstraint(linksArray_t &links);

	/**
	 * @brief Sets an local volume preservation constraints for
	 * tetrahedrons.
	 *
	 * @param[in] array of VolumeConstraints.
	 */
	void SetVolumeConstraint(volumeArray_t &volumes);

	/**
	 * @brief Indicate if SoftBody have to preserve volume globaly
	 *
	 * @param[in] global_volume object global volume if global_volume < 0 
	 * then volume will be precomputed from initial state.
	 */
	void SetGlobalVolumeConstraint(glm::float_t global_volume);

private:
	vec3Array_t                 *m_Particles;
    vec3Array_t                 mParticles;
    vec3Array_t                 mVelocities;
    vec3Array_t                 mForces;
// inverted mass of every particle
    floatArray_t				mMassInv;
    glm::float_t                mSpringiness;

    // constraints in soft body
    linksArray_t                mLinks;
	index3Array_t               mTriangles;
    volumeArray_t               mVolumes;

	//collision
	Sphere                      mBoundingSphere;

    // drawing data
	MeshData                    *m_Mesh;
    indexArray_t                mMeshVertexParticleMapping; /* Needed for updating mesh
															   vertex buffer */

friend class CPUSoftBodySolver;
friend class CUDASoftBodySolver;
friend class CUDAContext;
};

#endif
