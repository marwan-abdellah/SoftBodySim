#ifndef SB_ENGINE_PLANE_H_
#define SB_ENGINE_PLANE_H_

#include <glm/glm.hpp>

struct Plane {
	Plane(void) {}

	glm::vec3 normal;
	glm::vec3 origin;
};


#endif
