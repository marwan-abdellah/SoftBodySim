#ifndef SOFTBODY_H
#define SOFTBODY_H

//define gl math default precision as float - not 
#define GLM_PRECISION_MEDIUMP_FLOAT
#define GLM_PRECISION_MEDIUMP_UINT

#include "geometry/Arrays.h"
#include "Constraints.h"
#include "VertexBuffer.h"
#include "Body.h"

class SoftBody;
class CUDASoftBodySolver;


class SoftBody : Body {
public:
    SoftBody(glm::float_t mass, glm::float_t springness, glm::float_t damping,
             vec3Array_t *particles, index2Array_t *links, index4Array_t *volumes,
             vec2Array_t *textCoords, facesArray_t *faces,
             VertexBuffer::VertexBufferType);
    virtual ~SoftBody(void);

    const Mesh *getMesh(void) { return mMesh; }
	const Sphere *getBoundingVolume(void) { return &mCollisionSphere; }
	const ElementBuffer *getEdgesBuffer(void) { return mEdges; }

private:
	VertexBuffer *createGLVertexBuffer(vec3Array_t *vertexes, vec2Array_t *texCoords);
	ElementBuffer *createGLEdgeElementBuffer(index2Array_t *edges);
	ElementBuffer *createGLFacesElementBuffer(index3Array_t *faces);

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
	Sphere                       mCollisionSphere;

    // drawing data
    ElementBuffer               *mEdges;
	Mesh                      *mMesh;
    indexArray_t                mMeshVertexParticleMapping; /* Needed for updating mesh
															   vertex buffer */

friend class CUDASoftBodySolver;
};

#endif
