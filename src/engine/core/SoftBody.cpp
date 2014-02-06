#include "SoftBody.h"
#include "common.h"

using namespace glm;

SoftBody::SoftBody(float_t mass, float_t springness, float_t damping,
		vec3 *particles, unsigned int particles_count,
	 	uvec2 *links_indexes, unsigned int links_count,
	 	uvec4 *volumes_indexes, unsigned int volumes_count)
	:
		mVertexBuffer(0)
{
	mMassInv = 1.0/mass;
	mSpringiness = springness;
	mDamping = damping;
	mParticles.resize(particles_count);
	mVelocities.resize(particles_count);
	mForces.resize(particles_count);
	mLinks.resize(links_count);
	mVolumes.resize(volumes_count);

	for(unsigned int i = 0; i < particles_count; i++)
		mParticles.push_back(particles[i]);
	for(unsigned int i = 0; i < links_count; i++) {
		LinkConstraint lnk;
		lnk.index = links_indexes[i];
		lnk.restLength = length(mParticles[lnk.index[0]] - mParticles[lnk.index[1]]);
		mLinks.push_back(lnk);
	}
}

SoftBody::~SoftBody(void)
{
	if (mVertexBuffer) delete mVertexBuffer;
}

void SoftBody::createGLVertexBuffer(void)
{
	GLVertexBuffer *buf = new GLVertexBuffer(mParticles.size());
	buf->setVertexes(&mParticles[0]);
	mVertexBuffer = buf;
}

void SoftBody::initVertexBuffer(VertexBuffer::VertexBufferType type)
{
	switch(type) {
		case VertexBuffer::CPU_BUFFER:
			break;
		case VertexBuffer::OPENGL_BUFFER:
			createGLVertexBuffer();
			break;
	}
}

const VertexBuffer	*SoftBody::getVertexBuffer(void)
{
	return mVertexBuffer;
}
