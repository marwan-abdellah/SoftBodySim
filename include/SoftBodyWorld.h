#ifndef __SOFTBODY_WORLD_H
#define __SOFTBODY_WORLD_H

#include "SoftBodySolver.h"

class SoftBodyWorld {
	public:
	struct SoftBodyProperties {
		glm::vec3 	gravity;
	} properties;

	SoftBodyWorld(SoftBodyProperties&, SoftBodySolver &solver);
	~SoftBodyWorld(void);

	void addSoftBody(SoftBody *body);
	void addRigidBody(RigidBody *body);

	void simulationStep(float_t dt);
};

#endif
