#ifndef SBS_ENGINE_SPHERE_H_
#define SBS_ENGINE_SPHERE_H_

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
