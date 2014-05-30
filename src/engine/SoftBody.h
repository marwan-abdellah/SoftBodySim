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
class CUDACOntext;


struct Particle {
	glm::vec3 position;
	glm::vec3 projection;
	glm::vec3 velocity;
	glm::float_t imass; // inverted mass
};

class SoftBody : public Body {
public:
	typedef std::vector<Particle> particlesArray_t;
    SoftBody(glm::float_t mass, glm::float_t springness, glm::float_t damping,
             MeshData *mesh);
    ~SoftBody(void) {}

	const Sphere *getBoundingVolume(void) { return &mCollisionSphere; }

private:
	particlesArray_t            mParticles2;
    vec3Array_t                 mParticles;
    vec3Array_t                 mVelocities;
    vec3Array_t                 mForces;

    // inverted mass of every particle
    floatArray_t				mMassInv;
    glm::float_t                mDamping;
    glm::float_t                mSpringiness;

    // constraints in soft body
    linksArray_t                mLinks;
	index3Array_t               mTriangles;
    volumeArray_t               mVolumes;

	//collision
	Sphere                      mCollisionSphere;

    // drawing data
    indexArray_t                mMeshVertexParticleMapping; /* Needed for updating mesh
															   vertex buffer */

friend class CUDASoftBodySolver;
friend class CUDAContext;
};

#endif
