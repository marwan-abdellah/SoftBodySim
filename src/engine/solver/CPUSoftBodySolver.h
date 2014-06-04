#ifndef CPU_SOFT_BODY_SOLVER_H
#define CPU_SOFT_BODY_SOLVER_H

#include "engine/solver/SoftBodySolver.h"

class CPUSoftBodySolver : public SoftBodySolver {
public:
	/**
	 * Default constructor
	 */
	CPUSoftBodySolver(void);

	/**
	 * Default destructor
	 */
	~CPUSoftBodySolver(void);

	/**
	 * Simulate all bodies movement by dt time.
	 */
	void ProjectSystem(glm::float_t dt);

	bool Initialize(void);

	void Shutdown(void);

	void AddSoftBody(SoftBody *b);

	/**
	 * Update bodies drawing data
	 */
	void UpdateVertexBuffers(void);
private:
	struct SoftBodyDescriptor {
		SoftBody *body;
		int baseIdx; // index of first particle of body in mParticles array
		int count;   // number of particles for given body
		int linkBaseIdx;
		int linkCount;
		int mappingBaseIdx;
		int nMapping;
		struct {
			int descriptor;
		} shapeMatching;
	};
	struct ShapeDescriptor {
		glm::vec3 mc0; // mass center (mc0)
		vec3Array_t diffs; // relative locations (x0i - mc0);
		float_t radius; // maximum distance between mass center and particle;
	};
	typedef std::vector<SoftBodyDescriptor> descriptorsArray_t;
	typedef std::vector<ShapeDescriptor> shapeDescriptorsArray_t;

	vec3Array_t mPositions;
	vec3Array_t mProjections;
	vec3Array_t mVelocities;
	vec3Array_t mForces;
	floatArray_t mInvMasses;

	linksArray_t mLinks;
	indexArray_t mMapping;

	vec3Array_t mVertexes;

	descriptorsArray_t mDescriptors;
	shapeDescriptorsArray_t mShapes;

	void PredictMotion(float dt);
	void IntegrateSystem(float dt);
	void SolveGroundWallCollisions(void);
	void AddShapeDescriptor(SoftBody *obj);
	void SolveShapeMatchConstraint(void);
	bool mInitialized;
};


#endif
