#ifndef SB_PLANE_H
#define SB_PLANE_H

#include <glm/glm.hpp>

struct Plane {
	Plane(void) {}

	glm::vec3 m_normal;
	glm::vec3 m_origin;
};


#endif
