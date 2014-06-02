#ifndef SOFT_BODY_SOLVER_H
#define SOFT_BODY_SOLVER_H

#include "engine/SoftBody.h"
#include <list>

typedef std::list<SoftBody*> softbodyList_t; 

class SoftBodySolver {
public:
	/**
	 * Simulation parameters structure
	 *
	 * gravity - gravity vector (can be non-
	 */
	struct SoftBodyWorldParameters {
		glm::vec3   gravity;
		float_t     groundLevel;
		float_t     leftWall;
		float_t     rightWall;
		float_t     backWall;
		float_t     frontWall;
	};

	SoftBodySolver();
	virtual ~SoftBodySolver(void);

	virtual bool Initialize(void) = 0;
	virtual void Shutdown(void) = 0;
	virtual void ProjectSystem(glm::float_t dt) = 0;
	virtual void UpdateVertexBuffers(void) = 0;

	void SetWorldParameters(SoftBodyWorldParameters &params);
	const softbodyList_t &GetBodies(void) { return mBodies; }

	virtual void AddSoftBody(SoftBody *body);
	virtual void RemoveSoftBody(SoftBody *body);
protected:
	softbodyList_t   mBodies;
	int 			 mSolverSteps;
	SoftBodyWorldParameters mWorldParams;
};

#endif
