#ifndef SB_ENGINE_CPU_SOLVER_H_
#define SB_ENGINE_CPU_SOLVER_H_

#include <set>
#include "engine/solver/SoftBodySolver.h"
#include "engine/geometry/Plane.h"

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

	void GrabStart(Ray &r, glm::float_t radius, glm::float_t stifness);
	void GrabUpdate(Ray &r);
	void GrabStop();
private:
	struct ShapeDescriptor;
	struct SoftBodyDescriptor;
	struct ShapeRegion;
	typedef std::vector<SoftBodyDescriptor> descriptorsArray_t;
	typedef std::vector<ShapeDescriptor> shapeDescriptorsArray_t;
	typedef std::vector<ShapeRegion> shapeRegionsArray_t;
	typedef std::vector<glm::mat3> mat3Array_t;

	struct {
		int descriptor;
		bool enabled;
		indexArray_t particles;
		glm::vec3 destination;
		Plane dragPlane; // plan along which particles are dragged
		float_t stiffness;
	} mGrabbing;

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
	void AddShapeDescriptor(SoftBody *obj, int n);
	void SolveShapeMatchConstraint(void);
	void SolveFreezedParticlesConstraints();
	void GetRegion(int idx, const MeshData::neighboursArray_t &nei, int max, indexArray_t &out);
	void SolveVolumeConstraint();
	bool mInitialized;
	void CalculateMassCenters();
};

struct CPUSoftBodySolver::SoftBodyDescriptor {
	SoftBody *body;
	int baseIdx; // index of first particle of body in mParticles array
	int count;   // number of particles for given body
	int linkBaseIdx;
	int linkCount;
	int mappingBaseIdx;
	int nMapping;
	Sphere BS; // bounding sphere
	struct {
		glm::vec3 mc; // current mass center
		int descriptor;
	} shapeMatching;
	vec3Array_t posAccumulator; // needed for 
	indexArray_t accumulatorCounter; 
};

struct CPUSoftBodySolver::ShapeDescriptor {
	glm::vec3 mc0; // mass center (mc0)
	vec3Array_t initPos; // initial particle locations (x0i);
	float_t radius; // maximum distance between mass center and particle;
	float_t massTotal; // object total mass
	shapeRegionsArray_t regions; // shape regions
	float_t volume; // initial shape volume
};

struct CPUSoftBodySolver::ShapeRegion {
	glm::vec3 mc0; // initial region mass center
	float_t mass; // region total mass
	indexArray_t indexes; // indexes of neighbour particles creating region
};

#endif
