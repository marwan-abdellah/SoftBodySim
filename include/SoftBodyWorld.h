#ifndef __SOFTBODY_WORLD_H
#define __SOFTBODY_WORLD_H

class SoftBodyWorld {
	public:
	class SoftBodyProperties {
		glm::vec3 	gravity;
		
	};
	SoftBodyWorld(SoftBodyProperties&, SoftBodySolver &solver, SoftBodyRenderer &renderer);
	~SoftBodyWorld(void);

	void addSoftBody(SoftBody &body);
}

#endif
