#ifndef __SOFTBODY_H
#define __SOFTBODY_H

#include "VertexBuffer.h"

#include <glm/glm.hpp>
#include <vector>

//define gl math default precision -  float
#define GLM_PRECISION_MEDIUMP_FLOAT
#define GLM_PRECISION_MEDIUMP_UINT

class SoftBody;
class CUDASoftBodySolver;

struct CollisionBodyInfo {
    enum BOUNDING_TYPE {
        TRIANGLE,
        SPHERE,
    } type;
    union {
		glm::float_t ids[3];
		struct {
			glm::uint_t id;
			glm::float_t radius;
		} sphere;
    } data;
};

typedef std::vector<CollisionBodyInfo> collisionBodyInfoArray_t;

struct LinkConstraint {
    glm::uvec2          index;
    glm::float_t        restLength;
	glm::float_t		stiffness;
};

struct VolumeConstraint {
    glm::uvec4          index;
    glm::float_t        restLength;
};

struct CollisionConstraint {
    glm::uint_t			vertId;
	glm::vec3			collNormal;
	glm::vec3			entryPoint;
};

typedef std::vector<glm::vec3>              vec3Array_t;
typedef std::vector<glm::vec2>              vec2Array_t;
typedef std::vector<glm::float_t>           floatArray_t;
typedef std::vector<glm::uint>              indexArray_t;
typedef std::vector<glm::uvec3>             index3Array_t;
typedef std::vector<glm::uvec2>             index2Array_t;
typedef std::vector<glm::uvec3>             index4Array_t;
typedef std::vector<LinkConstraint>         linksArray_t;
typedef std::vector<VolumeConstraint>       volumeArray_t;

typedef std::vector< std::vector<glm::uvec2>  > facesArray_t;

typedef struct Mesh {
    VertexBuffer                 *vertexes;
    ElementBuffer                *faces;
    ElementBuffer                *edges;
} Mesh_t;

class SoftBody {
public:
    SoftBody(glm::float_t mass, glm::float_t springness, glm::float_t damping,
             vec3Array_t *particles, index2Array_t *links, index4Array_t *volumes,
             vec2Array_t *textCoords, facesArray_t *faces,
             VertexBuffer::VertexBufferType);
    ~SoftBody(void);

    const Mesh_t *getMesh(void) { return mMesh; }
private:
    Mesh_t *createGLVertexBufferMesh(vec3Array_t *vertexes, vec2Array_t *texCoords,
                                     index2Array_t *edges, index3Array_t *faces);

    vec3Array_t                 mParticles;
    vec3Array_t                 mVelocities;
    vec3Array_t                 mForces;

    // inverted mass of every particle
    floatArray_t				mMassInv;
    glm::float_t                mDamping;
    glm::float_t                mSpringiness;

    // constraints in soft body
    linksArray_t                mLinks;
    volumeArray_t               mVolumes;

	// collision detection
	collisionBodyInfoArray_t	mCollisionBodies;

    // drawing data
    Mesh_t                      *mMesh;
    indexArray_t                mMeshVertexParticleMapping; /* Needed for updating mesh vertex buffer */

friend class CUDASoftBodySolver;
};

#endif
