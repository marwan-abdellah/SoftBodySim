#ifndef SB_SPHERE_H
#define SB_SPHERE_H

#include <glm/glm.hpp>

struct Sphere {
	Sphere(void) : mCenter(glm::vec3()), mRadius(0) {}
	Sphere(const Sphere &s)
		: mCenter(s.mCenter), mRadius(s.mRadius) {}
	~Sphere(void) {}

	glm::vec3    mCenter;
	glm::float_t mRadius;
};

#endif
