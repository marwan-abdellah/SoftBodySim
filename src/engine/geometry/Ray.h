#ifndef RAY_H
#define RAY_H

#include <glm/fwd.hpp>

struct Ray {
	public:
		Ray(const glm::vec3 &o, const glm::vec3 &d)
			: origin(o), direction(glm::normalize(d)) {}
		Ray(const Ray &r) : origin(r.origin), direction(r.direction) {}
		glm::vec3 origin;
		glm::vec3 direction;
};

#endif
