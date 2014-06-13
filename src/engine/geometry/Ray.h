#ifndef SB_ENGINE_RAY_H_
#define SB_ENGINE_RAY_H_

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
