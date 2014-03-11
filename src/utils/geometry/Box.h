#ifndef SB_BOX_H
#define SB_BOX_H

struct Box {
	Box(void) {}
	Box(const Box &p)
		: m_bottomLeft(p.m_bottomLeft), m_upperRight(p.m_upperRight) {}
	~Box(void) {}
	glm::vec3 m_bottomLeft;
	glm::vec3 m_upperRight;
};

#endif
