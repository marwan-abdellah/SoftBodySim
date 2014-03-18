#ifndef SB_CUBE_H
#define SB_CUBE_H

#include <glm/glm.hpp>

/*
 * Creates an cube aligned with x, y, z axis
 */
struct Cube {
	Cube(glm::vec3 bl, glm::vec3 ur) :
		m_bottomLeft(bl), m_upperRight(ur)
	{}
	Cube(const Cube &p)
		: m_bottomLeft(p.m_bottomLeft), m_upperRight(p.m_upperRight) {}
	~Cube(void) {}
	glm::vec3 m_bottomLeft;
	glm::vec3 m_upperRight;
};

#endif
