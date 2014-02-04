#ifndef __SOFTBODY_H
#define __SOFTBODY_H


#include <glm/glm.hpp>
#include <vector>

class SoftBody;
class CUDASoftBodySolver;

///gclass CollisionBody {
///g
///gpublic:
///g	enum {
///g		PLANE,
///g		SPHERE,
///g	} BoundingType;
///g
///g	CollisionBody(BoundingType &bt);
///g	~CollisionBody(void);
///g
///g	BoundingType getBoundingType(void);
///g	bool 		 checkCollision(CollisionBody &body) = 0;
///g	glm::vec3	 collisionPoint(CollisionBody &body) = 0;
///g
///g	union {
///g		struct {
///g			glm::vec3 origin;
///g			glm::vec3 normal;
///g		} plane;
///g		struct {
///g			glm::vec3 origin;
///g			glm::float_t radius;
///g		} sphere,
///g	} data;
///g	glm::float_t offset;
///g};
///g
///g

class SoftBody {
public:

	struct Links {
		glm::uvec2 		indexes;
		glm::float_t	restLength;
	};

	struct Volumes {
		glm::uvec4 		indexes;
		glm::float_t 	restLength;
	};

	SoftBody(glm::vec3 *particles, unsigned int particles_count,
			 glm::uvec2 *links_indexes, unsigned int links_count,
			 glm::uvec4 *volumes_indexes, unsigned int volumes_count);

	~SoftBody(void);

	std::vector<glm::vec3>	 	m_vertexes;
	std::vector<glm::vec3>	 	m_velocities;
	std::vector<glm::vec3>	 	m_forces;

	// inverted mass of every particle
	float_t						m_mass_inv;

	// constraints in soft body
	std::vector<Links>		 	m_links;
	std::vector<Volumes>		m_volumes;
};


#endif
