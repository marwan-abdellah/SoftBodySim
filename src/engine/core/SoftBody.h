#ifndef __SOFTBODY_H
#define __SOFTBODY_H

#include "common.h"
#include "VertexBuffer.h"

#include <glm/glm.hpp>
#include <vector>

class SoftBody;
class CUDASoftBodySolver;

///gclass CollisionBody {
///g
///gpublic:
///g    enum {
///g        PLANE,
///g        SPHERE,
///g    } BoundingType;
///g
///g    CollisionBody(BoundingType &bt);
///g    ~CollisionBody(void);
///g
///g    BoundingType getBoundingType(void);
///g    bool          checkCollision(CollisionBody &body) = 0;
///g    glm::vec3     collisionPoint(CollisionBody &body) = 0;
///g
///g    union {
///g        struct {
///g            glm::vec3 origin;
///g            glm::vec3 normal;
///g        } plane;
///g        struct {
///g            glm::vec3 origin;
///g            glm::float_t radius;
///g        } sphere,
///g    } data;
///g    glm::float_t offset;
///g};
///g
///g
//

struct LinkConstraint {
    glm::uvec2          index;
    glm::float_t        restLength;
};

struct VolumeConstraint {
    glm::uvec4          index;
    glm::float_t        restLength;
};

typedef std::vector<glm::vec3>              vec3Array_t;
typedef std::vector<LinkConstraint>         linksArray_t;
typedef std::vector<VolumeConstraint>       volumeArray_t;

class SoftBody {
public:
    SoftBody(glm::float_t mass, glm::float_t springness, glm::float_t damping,
             glm::vec3 *particles, unsigned int particles_count,
             glm::uvec2 *links_indexes, unsigned int links_count,
             glm::uvec4 *volumes_indexes, unsigned int volumes_count);
    ~SoftBody(void);

    void                        initVertexBuffer(VertexBuffer::VertexBufferType);
    const VertexBuffer          *getVertexBuffer(void);
private:
    void createGLVertexBuffer(void);

    vec3Array_t                 mParticles;
    vec3Array_t                 mVelocities;
    vec3Array_t                 mForces;

    // inverted mass of every particle
    glm::float_t                mMassInv;
    glm::float_t                mDamping;
    glm::float_t                mSpringiness;

    // constraints in soft body
    linksArray_t                mLinks;
    volumeArray_t               mVolumes;

    // buffer object
    VertexBuffer                *mVertexBuffer;

friend class CUDASoftBodySolver;
};

#endif
