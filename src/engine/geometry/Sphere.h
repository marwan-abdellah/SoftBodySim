#ifndef SB_SPHERE_H
#define SB_SPHERE_H

#include <glm/glm.hpp>

struct Sphere {
	Sphere(void) {}
	Sphere(const Sphere &s)
		: m_center(s.m_center), m_radius(s.m_radius) {}
	~Sphere(void) {}

	glm::vec3    m_center;
	glm::float_t m_radius;
};

#endif
