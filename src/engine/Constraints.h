#ifndef SB_CONSTRAINTS_H
#define SB_CONSTRAINTS_H

#include <glm/glm.hpp>
#include <vector>

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
    glm::float_t        restLength;
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

#endif
