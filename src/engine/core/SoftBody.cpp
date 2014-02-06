#include "SoftBody.h"

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
