#ifndef SOFTBODY_H
#define SOFTBODY_H

//define gl math default precision as float - not 
#define GLM_PRECISION_MEDIUMP_FLOAT
#define GLM_PRECISION_MEDIUMP_UINT

#include "geometry/Arrays.h"
#include "Constraints.h"
#include "Body.h"
#include "model/MeshData.h"

class SoftBody;
class CUDASoftBodySolver;


class SoftBody : public Body {
public:
    SoftBody(glm::float_t mass, glm::float_t springness, glm::float_t damping,
             MeshData *mesh);
    ~SoftBody(void) {}

	const Sphere *getBoundingVolume(void) { return &mCollisionSphere; }

private:
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

	//collision
	Sphere                      mCollisionSphere;

    // drawing data
    indexArray_t                mMeshVertexParticleMapping; /* Needed for updating mesh
															   vertex buffer */

friend class CUDASoftBodySolver;
};

#endif
