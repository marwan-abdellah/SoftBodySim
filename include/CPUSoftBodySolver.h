#ifndef __CPU_SOFTBODY_H
#define __CPU_SOFTBODY_H

class CPUBodySoftSolver {
	public:
		void 	predictMotion(glm::float_t dt) = 0;
		void 	updateVelocities(glm::float_t dt) = 0;
		void 	solveCollisions(glm::float_t dt) = 0;
		void 	solveLinks(glm::float_t dt) = 0;
		void 	synchronizeData(SoftBody &body, VertexBuffer);
};

#endif
