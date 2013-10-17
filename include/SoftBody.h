#ifndef __SOFTBODY_H
#define __SOFTBODY_H


class CollisionBody {
	enum {
		PLANE,
		SPHERE,
	} BoundingType;

	CollisionBody(BoundingType &bt);
	~CollisionBody(void);

	BoundingType getBoundingType(void);
	bool 		 checkCollision(CollisionBody &body) = 0;
	glm::vec3	 collisionPoint(CollisionBody &body) = 0;

	union {
		struct {
			glm::vec3 origin;
			glm::vec3 normal;
		} plane;
		struct {
			glm::vec3 origin;
			glm::float_t radius;
		} sphere,
	} data;
	glm::float_t offset;
};


class SoftBody {
public:

	class Links {
		std::vector<glm::uint2> 	m_indexes;
		std::vector<glm::float_t> 	m_restLength;
	};

	class Volumes {
		std::vector<glm::uint4> 	m_indexes;
		std::vector<glm::float_t> 	m_restLength;
	};

	SoftBody(const char *file);
	~SoftBody(void);

	void	setSolver(SoftBodySolver &solver);

	const VertexBuffer &getVertexBuffer(VertexBuffer.type);
	const IndexBuffer &getLinksConnections(IndexBuffer.type);

protected:
	VertexBuffer				m_vertexBuffer;
	std::vector<glm::vec3>	 	m_vertexes;
	std::vector<glm::vec3>	 	m_projections;
	std::vector<glm::vec3>	 	m_velocities;
	std::vector<glm::vec3>	 	m_forces;

	float_t						m_mass_inv;

	// constraints in soft body model
	Links					 	m_links;
	Volumes						m_volumes;
};




#endif
