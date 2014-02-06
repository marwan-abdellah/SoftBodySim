#include "SoftBody.h"
#include "common.h"
#include <cstring>

using namespace glm;

SoftBody::SoftBody(float_t mass, float_t springness, float_t damping,
		const vec3 *particles, unsigned int particles_count,
	 	const uvec2 *links_indexes, unsigned int links_count,
	 	const uvec4 *volumes_indexes, unsigned int volumes_count)
{
	mMassInv = 1.0/mass;
	mSpringiness = springness;
	mDamping = damping;
	mParticles.resize(particles_count);
	mVelocities.resize(particles_count);
	mForces.resize(particles_count);
	mLinks.resize(links_count);
	mVolumes.resize(volumes_count);

    memset(&mMesh, 0x0, sizeof(Mesh_t));

	for(unsigned int i = 0; i < particles_count; i++)
		mParticles[i] = particles[i];
	for(unsigned int i = 0; i < links_count; i++) {
		LinkConstraint lnk;
		lnk.index = links_indexes[i];
		lnk.restLength = length(mParticles[lnk.index[0]] - mParticles[lnk.index[1]]);
		mLinks[i] = lnk;
	}
}

SoftBody::~SoftBody(void)
{
    if (!mMesh) return;
    if (mMesh->vertexes) delete mMesh->vertexes;
    if (mMesh->faces) delete mMesh->faces;
    if (mMesh->edges) delete mMesh->edges;
    delete mMesh;
}

void SoftBody::createGLVertexBuffer(void)
{
	GLVertexBuffer *buf = new GLVertexBuffer(mParticles.size());

    if (mMesh->vertexes) delete mMesh->vertexes;
	buf->setVertexes(&(mParticles[0]));
	mMesh->vertexes = buf;
}

void SoftBody::initVertexBuffers(VertexBuffer::VertexBufferType type)
{
    if (!mMesh) mMesh = new Mesh_t;

	switch(type) {
		case VertexBuffer::CPU_BUFFER:
			break;
		case VertexBuffer::OPENGL_BUFFER:
			createGLVertexBuffer();
			break;
	}
}
