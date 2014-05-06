#ifndef SB_BOX_H
#define SB_BOX_H

#include <glm/glm.hpp>

/*
 * Creates an Box aligned with x, y, z axis
 */
struct Box {
	Box(glm::vec3 bl, glm::vec3 ur) :
		m_bottomLeft(bl), m_upperRight(ur)
	{}
	Box(const Box &p)
		: m_bottomLeft(p.m_bottomLeft), m_upperRight(p.m_upperRight) {}
	~Box(void) {}
	glm::vec3 m_bottomLeft;
	glm::vec3 m_upperRight;
};

#endif
