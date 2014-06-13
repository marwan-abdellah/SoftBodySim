#include "engine/solver/CPUSoftBodySolver.h"
#include "engine/geometry/Math.h"
#include "common.h"

#include <queue>
#include <set>
#include <algorithm>

#include <glm/ext.hpp>
#include <glm/gtx/intersect.hpp>

using namespace glm;

static vec3 calculateMassCenter(vec3 *pos, float_t *mass, int n);
static float_t calculateVolume(glm::vec3 *pos, glm::uvec3 *triangles, glm::vec3 *norms, uint_t *accum, int n);

static inline float_t triangle_area(glm::vec3 &a, glm::vec3 &b, glm::vec3 &c)
{
	glm::vec3 ab = b - a;
	glm::vec3 ac = c - a;
	return 0.5f * glm::length(cross(ab, ac));
}

static float_t calculateVolume(glm::vec3 *pos, glm::uvec3 *triangles, glm::vec3 *norms, uint_t *accum, int n)
{
	double ret = 0.0f;
	glm::vec3 norm;

	for(int i = 0; i < n; i++) {
		glm::vec3 v0 = pos[triangles[i][0]];
		glm::vec3 v1 = pos[triangles[i][1]];
		glm::vec3 v2 = pos[triangles[i][2]];
		norm = normalize(glm::cross(v1 - v0, v2 - v0));
		float_t area = triangle_area(v0, v1, v2);
		ret += area * glm::dot(v0 + v1 + v2, norm);

		if (norms) {
			norm = area * norm;
			norms[triangles[i][0]] += norm;
			norms[triangles[i][1]] += norm;
			norms[triangles[i][2]] += norm;
			accum[triangles[i][0]]++;
			accum[triangles[i][1]]++;
			accum[triangles[i][2]]++;
		}
	}

	return (float_t)ret / 3.0f;
}

CPUSoftBodySolver::CPUSoftBodySolver()
{
}

CPUSoftBodySolver::~CPUSoftBodySolver()
{
	Shutdown();
}

void CPUSoftBodySolver::PredictMotion(float dt)
{
	FOREACH_R(it, mGrabbing.particles) {
		vec3 g = (mGrabbing.destination - mPositions[*it]) * mGrabbing.stiffness;
		mForces[*it] = g;
	}

	REP(i, mPositions.size()) {
		mVelocities[i] += dt * mInvMasses[i] * (mForces[i] + mWorldParams.gravity);
		mVelocities[i] *= 0.99f;
		mProjections[i] = mPositions[i] + mVelocities[i] * dt;
	}
}

void CPUSoftBodySolver::GrabStart(Ray &ray, float_t radius, float_t stifness)
{
	if (mGrabbing.enabled) return;
	int descr = -1;
	vec3 ip;
	vec3 nr;

	// broad phase
	// FIXME check distance from ray origin also
	FOREACH_R(it, mDescriptors) {
		vec3 center = it->shapeMatching.mc;
		float_t rad = mShapes[it->shapeMatching.descriptor].radius;
		if (glm::intersectRaySphere(ray.origin, ray.direction,
					center, rad, ip, nr)) {
			descr = std::distance(mDescriptors.begin(), it);
			break;
		}
	}
	if (descr == -1)
		return;

	// narrow phase
	for(int i = mDescriptors[descr].baseIdx;
			i < mDescriptors[descr].baseIdx + mDescriptors[descr].count; i++)
	{
		if (glm::intersectRaySphere(ray.origin, ray.direction,
					mPositions[i], radius, ip, nr)) {
			mGrabbing.particles.push_back(i);
		}
	}

	DBG("Grabbed %ld paricles", mGrabbing.particles.size());

	mGrabbing.dragPlane.origin = mDescriptors[descr].shapeMatching.mc;
	mGrabbing.dragPlane.normal = - ray.direction;
	mGrabbing.descriptor = descr;
	mGrabbing.enabled = true;
	mGrabbing.stiffness = stifness;

	GrabUpdate(ray);
}

void CPUSoftBodySolver::GrabUpdate(Ray &ray)
{
	float_t dist;
	if (mGrabbing.enabled) {
		if (intersectRayPlane(ray.origin, ray.direction,
			mGrabbing.dragPlane.origin, mGrabbing.dragPlane.normal, dist))
		{
			mGrabbing.destination = ray.origin + dist * ray.direction;
		}
	}
}

void CPUSoftBodySolver::GrabStop()
{
	if (mGrabbing.enabled) {
		mGrabbing.enabled = false;
		FOREACH_R(it, mGrabbing.particles)
			mForces[*it] = vec3(0.0f, 0.0f, 0.0f);
		mGrabbing.particles.clear();
	}
}

void CPUSoftBodySolver::IntegrateSystem(float dt)
{
	REP(i, mPositions.size()) {
		mVelocities[i] = (mProjections[i] - mPositions[i]) / dt;
		//mPositions[i] = mProjections[i];
		mPositions[i] = mPositions[i] + dt * mVelocities[i];
	}
}

void CPUSoftBodySolver::SolveGroundWallCollisions(void)
{
	float_t ground = mWorldParams.groundLevel;
	float_t left = mWorldParams.leftWall;
	float_t right = mWorldParams.rightWall;
	float_t front = mWorldParams.frontWall;
	float_t back = mWorldParams.backWall;

	REP(i, mPositions.size()) {
		mProjections[i][1] = mProjections[i][1] < ground ? ground : mProjections[i][1];
		mProjections[i][0] = mProjections[i][0] < left ? left : mProjections[i][0];
		mProjections[i][0] = mProjections[i][0] > right ? right : mProjections[i][0];
		mProjections[i][2] = mProjections[i][2] > front ? front : mProjections[i][2];
		mProjections[i][2] = mProjections[i][2] < back ? back : mProjections[i][2];
	}
}

#define ADAPTIVE_SH
#ifdef ADAPTIVE_SH
void CPUSoftBodySolver::SolveShapeMatchConstraint(void)
{
	mat3 R, S;
	vec3 mc;
	FOREACH_R(it, mDescriptors) {
		// clear accumulator(s)
		REP(i, it->count) {
			it->posAccumulator[i] = vec3(0,0,0);
			it->accumulatorCounter[i] = 0;
		}

		// process regions
		FOREACH_R(reg, mShapes[it->shapeMatching.descriptor].regions) {
			mat3 A(0);
			mc = vec3(0,0,0);

			// calculate regular particles
			FOREACH_R(k, reg->indexes) {
				mc += mProjections[it->baseIdx + *k] *
					mInvMasses[it->baseIdx + *k];
				A += mInvMasses[it->baseIdx + *k] *
					outerProduct(mProjections[it->baseIdx + *k],
							mShapes[it->shapeMatching.descriptor].initPos[*k]);
			}

			// current region mass center
			mc = mc / (reg->mass);
			A -= reg->mass * outerProduct(mc, reg->mc0);

			// calculate Rotation matrix
			polar_decomposition(A, R, S);

			// accumulate newly calculated positions
			FOREACH_R(idx, reg->indexes) {
				vec3 g = R * (mShapes[it->shapeMatching.descriptor].initPos[*idx] -
						reg->mc0) + mc;
				it->posAccumulator[*idx] += g;
				it->accumulatorCounter[*idx]++;
			}
		}

		// update particles position by taking average from region transform
		float_t sping = it->body->mSpringiness;
		REP(i, it->count) {
			vec3 g = it->posAccumulator[i] / (float_t)it->accumulatorCounter[i];
			mProjections[it->baseIdx + i] += (g - mProjections[it->baseIdx + i]) * sping;
		}

	}
}

#else
void CPUSoftBodySolver::SolveShapeMatchConstraint(void)
{
	vec3 mc;
	mat3 A, R, S;
	float_t sping;
	FOREACH_R(it, mDescriptors) {
		// calculate A = sum(mi * (xi - mc) * (x0i - mc0))
		A = mat3(0.0f);
		REP(i, it->count) {
			A += mInvMasses[it->baseIdx + i] *
				outerProduct(mProjections[it->baseIdx + i],
						mShapes[it->shapeMatching.descriptor].initPos[i]);
		}
		A -= mShapes[it->shapeMatching.descriptor].massTotal *
			outerProduct(mc, mShapes[it->shapeMatching.descriptor].mc0);

		polar_decomposition(A, R, S);

		// calculate target positions and multiply it be constraint stiffness
		sping = it->body->mSpringiness;
	    REP(i , it->count) {
			vec3 g = R * (mShapes[it->shapeMatching.descriptor].initPos[i] -
					mShapes[it->shapeMatching.descriptor].mc0) + mc;
			mProjections[it->baseIdx + i] += (g - mProjections[it->baseIdx + i]) * sping;
		}

		it->shapeMatching.mc = mc;
		it->body->mBS.mCenter = mc;
	}
}
#endif

void CPUSoftBodySolver::CalculateMassCenters()
{
	vec3 mc;
	FOREACH_R(it, mDescriptors) {
		// calculate mass center after pojection
		mc = calculateMassCenter(&mProjections[it->baseIdx],
								 &mInvMasses[it->baseIdx],
								 it->count);
		it->shapeMatching.mc = mc;
		it->body->mBS.mCenter = mc;
	}
}

void CPUSoftBodySolver::SolveVolumeConstraint()
{
	float_t current_volume, rest_volume;;

	FOREACH_R(it, mDescriptors) {
		REP(k, it->count) {
			it->accumulatorCounter[k] = 0;
			it->posAccumulator[k] = vec3(0,0,0);
		}
		rest_volume = mShapes[it->shapeMatching.descriptor].volume;

		// calculate current volume and normals sum
		current_volume = calculateVolume(&mPositions[it->baseIdx], &(it->body->mTriangles[0]), &(it->posAccumulator[0]), &(it->accumulatorCounter[0]), it->body->mTriangles.size());
		float_t diff = (current_volume - rest_volume) / rest_volume;
		if (diff > -0.01f) return;
		ERR("Volume diff: %f %", diff);
		// update positions
		REP(k, it->count)
			mProjections[it->baseIdx + k] -= 0.1f * diff * it->posAccumulator[k] / (float_t)it->accumulatorCounter[k];
	}
}

void CPUSoftBodySolver::ProjectSystem(float_t dt)
{
	if (!mInitialized) return;

	PredictMotion(dt);

	CalculateMassCenters();
	SolveShapeMatchConstraint();
	SolveVolumeConstraint();
	SolveGroundWallCollisions();

	IntegrateSystem(dt);
}

bool CPUSoftBodySolver::Initialize(void)
{
	FOREACH_R(it, mBodies)
		AddSoftBody(*it);

	mInitialized = true;
	return mInitialized;
}

void CPUSoftBodySolver::Shutdown(void)
{
	mPositions.clear();
	mProjections.clear();
	mForces.clear();
	mVelocities.clear();
	mInvMasses.clear();
	mLinks.clear();
	mMapping.clear();
	mDescriptors.clear();
	mShapes.clear();

	mGrabbing.enabled = false;
	mGrabbing.particles.clear();

	SoftBodySolver::Shutdown();
	mInitialized = false;
}

static vec3 calculateMassCenter(vec3 *pos, float_t *mass, int n)
{
	double masssum = 0.0;
	vec3 xmsum = vec3(0,0,0);

	//calculate sum(xi * mi) amd sum(mi)
	REP(i, n) {
		xmsum += pos[i] * mass[i];
		masssum += mass[i];
	}

	return xmsum / (float_t)masssum;
}

void CPUSoftBodySolver::GetRegion(int idx, const MeshData::neighboursArray_t &nei, int max, indexArray_t &out)
{
	struct Node {
		Node(int i, int d) : idx(i), distance(d) {}
		int idx;
		int distance;
	};
	std::queue<Node> toprocess;
	std::set<int> processed;

	toprocess.push(Node(idx, 0));

	while (!toprocess.empty()) {
		Node n = toprocess.front();
		out.push_back(n.idx);
		toprocess.pop();
		processed.insert(n.idx);

		if (n.distance >= max) return;

		FOREACH_R(it, nei[n.idx]) {
			if (processed.find(*it) == processed.end())
				toprocess.push(Node(*it, n.distance + 1));
		}
	}
}

void CPUSoftBodySolver::AddShapeDescriptor(SoftBody *obj, int distance)
{
	ShapeDescriptor ret;
	float_t max = 0;
	float_t mass = 0.0f;

	ret.mc0 = calculateMassCenter(
			&(obj->mParticles[0]), &(obj->mMassInv[0]), obj->mParticles.size());

	ret.initPos.reserve(obj->mParticles.size());

	// add initial locations x0i
	REP(i, obj->mParticles.size()) {
		vec3 q = obj->mParticles[i] - ret.mc0;
		ret.initPos.push_back(obj->mParticles[i]);
		mass += obj->mMassInv[i];
		if (length(q) > max)
			max = length(q);
	}
	ret.radius = max;
	ret.massTotal = mass;
	ret.regions.reserve(obj->mParticles.size());

	const MeshData::neighboursArray_t &na = obj->mMesh->GetNeighboursArray();
	long len = 0;
	unsigned int smin = 999999;
	unsigned int smax = 0;

	// create shape regions
	REP(i, obj->mParticles.size()) {
		ShapeRegion reg;
		mass = 0.0f;
		vec3 mc(0,0,0);
		float_t len = 0.0f;
		vec3 norm(0,0,0);

		GetRegion(i, na, distance, reg.indexes);

		len += reg.indexes.size();
		if (smin > reg.indexes.size())
			smin = reg.indexes.size();
		if (smax < reg.indexes.size())
			smax = reg.indexes.size();

		FOREACH_R(it, reg.indexes) {
			mass += obj->mMassInv[*it];
			mc += obj->mParticles[*it] * obj->mMassInv[*it];
		}
		FOREACH_R(it, reg.indexes)
			len += obj->mMassInv[*it] * glm::length(obj->mParticles[*it] - mc);

		reg.mass = mass;
		reg.mc0 = mc / mass;
		ret.regions.push_back(reg);
	}

	ret.volume = calculateVolume(&(obj->mParticles[0]), &(obj->mTriangles[0]), NULL, NULL, obj->mTriangles.size()); 
	ERR("Rest Volume :%f", ret.volume);

	WRN("Regions total: %ld", ret.regions.size());
	WRN("Average region size: %f", (float)len / ret.regions.size());
	WRN("Max region size: %d", smax);
	WRN("Min region size: %d", smin);

	mShapes.push_back(ret);
}

void CPUSoftBodySolver::UpdateVertexBuffers(void)
{
	if (!mInitialized) return;

	FOREACH(it, &mDescriptors) {
		for(int i = 0; i < it->nMapping; i++) {
			int idx = mMapping[it->mappingBaseIdx + i];
			mVertexes[i] = mPositions[it->baseIdx + idx];
		}
		VertexBuffer *vb = it->body->GetVertexes();
		vb->SetVertexes(&mVertexes[0]);
	}
}

void CPUSoftBodySolver::AddSoftBody(SoftBody *b)
{
	SoftBodyDescriptor descr;

	descr.body = b;
	descr.baseIdx = mPositions.size();
	descr.count = b->mParticles.size();
	descr.linkBaseIdx = mLinks.size();
	descr.linkCount = b->mLinks.size();
	descr.mappingBaseIdx = mMapping.size();
	descr.nMapping = b->mMeshVertexParticleMapping.size();
	descr.shapeMatching.descriptor = mShapes.size();
	descr.posAccumulator.resize(descr.count);
	descr.accumulatorCounter.resize(descr.count);

	AddShapeDescriptor(b, 2);

	b->mBS.mCenter = mShapes[descr.shapeMatching.descriptor].mc0;
	b->mBS.mRadius = mShapes[descr.shapeMatching.descriptor].radius;
	descr.shapeMatching.mc = mShapes[descr.shapeMatching.descriptor].mc0;

	mDescriptors.push_back(descr);

	mVertexes.reserve(b->mMeshVertexParticleMapping.size());
	mPositions.insert(mPositions.end(), b->mParticles.begin(), b->mParticles.end());
	mProjections.insert(mProjections.end(), b->mParticles.begin(), b->mParticles.end());
	mVelocities.resize(mVelocities.size() + b->mParticles.size());
	mForces.resize(mPositions.size() + b->mParticles.size());
	mInvMasses.insert(mInvMasses.end(), b->mMassInv.begin(), b->mMassInv.end());

	mLinks.insert(mLinks.end(), b->mLinks.begin(), b->mLinks.end());
	mMapping.insert(mMapping.end(), b->mMeshVertexParticleMapping.begin(),
			b->mMeshVertexParticleMapping.end());

	mVertexes.resize(mVertexes.capacity());

	SoftBodySolver::AddSoftBody(b);
}
