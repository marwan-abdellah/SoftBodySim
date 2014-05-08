#include "MeshData.h"
#include "common.h"

#include <map>

using namespace glm;
using namespace std;

class uindx3Comp {
public:
	uindx3Comp(size_t y, size_t z) : m_y(y), m_z(z) {}
	uindx3Comp(const uindx3Comp &c) : m_y(c.m_y), m_z(c.m_z) {}
	bool operator()(const glm::uvec3 &a, const glm::uvec3 &b) { return a[0] * m_y * m_z + a[1] * m_z + a[2] > b[0] * m_y * m_z + b[1] * m_z + b[2] ; }
private:
	size_t m_y, m_z;
};

MeshData MeshData::CreateCube(const Box &c, size_t nx, size_t ny, size_t nz)
{
	SB_ASSERT(((nx > 1) && (ny > 1) && (ny > 1)));

	MeshData ret;

	index3Array_t grid;
	uindx3Comp comp(ny, nz);
	map<uvec3, vec3, uindx3Comp> vertexMap(comp);

	vec3 diff = c.m_upperRight - c.m_bottomLeft;

	vec3 xdiff = 1.0f / (nx - 1) * vec3(diff[0], 0, 0);
	vec3 ydiff = 1.0f / (ny - 1) * vec3(0, diff[1], 0);
	vec3 zdiff = 1.0f / (nz - 1) * vec3(0, 0, diff[2]);

#define ADD_FACE_VERTEXES(xs, xe, ys, ye, zs, ze) \
	grid.clear(); \
	for (unsigned int x = xs; x < xe; x++) \
		for (unsigned int y = ys; y < ye; y++) \
			for (unsigned int z = zs; z < ze; z++) {\
				uvec3 ids(x, y, z);\
				vec3 v = (float_t)x * xdiff + (float_t)y * ydiff + (float_t)z * zdiff;\
				v += c.m_bottomLeft; \
				vertexMap.insert( std::pair<glm::uvec3, glm::vec3>(ids, v) ); \
				grid.push_back(ids); \
			}\

#define ADD_FACE(a, b, c) {\
	uvec3 vid1 = grid[a];\
	uvec3 vid2 = grid[b];\
	uvec3 vid3 = grid[c];\
	map<uvec3, vec3>::iterator it1 = vertexMap.find(vid1); \
	map<uvec3, vec3>::iterator it2 = vertexMap.find(vid2); \
	map<uvec3, vec3>::iterator it3 = vertexMap.find(vid3); \
	unsigned int d1 = std::distance(vertexMap.begin(), it1); \
	unsigned int d2 = std::distance(vertexMap.begin(), it2); \
	unsigned int d3 = std::distance(vertexMap.begin(), it3); \
	ret.faces.push_back(uvec3(d1, d2, d3));\
}\
	
#define ADD_FACES(w, h) \
	for (unsigned int x = 0; x < (w); x++) \
		for (unsigned int y = 0; y < (h); y++) { \
			int idx = y + (h) * x; \
			if ((x < (w) - 1) && (y < (h) - 1)) { \
				ADD_FACE(idx, idx + h + 1, idx + 1); \
				ADD_FACE(idx, idx + h, idx + h + 1); \
			}\
		}

	// back
	ADD_FACE_VERTEXES(0, nx, 0, ny, 0, 1);
	ADD_FACES(nx, ny);

	// front
	ADD_FACE_VERTEXES(0, nx, 0, ny, nz - 1, nz);
	ADD_FACES(nx, ny);

	// top
	ADD_FACE_VERTEXES(0, nx, ny - 1, ny, 0, nz);
	ADD_FACES(nx, nz);

	// bottom
	ADD_FACE_VERTEXES(0, nx, 0, 1, 0, nz);
	ADD_FACES(nx, nz);

	// left
	ADD_FACE_VERTEXES(0, 1, 0, ny, 0, nz);
	ADD_FACES(ny, nz);

	// right
	ADD_FACE_VERTEXES(nx - 1, nx, 0, ny, 0, nz);
	ADD_FACES(ny, nz);

	FOREACH_R(it, vertexMap) {
		Vertex v(it->second, vec2(0,0), vec3(0,0,0));
		ret.vertexes.push_back(v);
	}
	return ret; // by value - assume RVO
}

MeshData MeshData::CreatePlane(float width, float height, size_t nx, size_t ny)
{
	SB_ASSERT((nx > 1) && (ny > 1) && (width > 0.0) && (height > 0.0));
	MeshData ret;

	const float xstep = width / (nx - 1);
	const float ystep = height / (ny - 1);

	vec3 shift(width / 2.0, height / 2.0, 0.0);

	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			vec3 pos(i * xstep, j * ystep, 0.0);
			pos = pos - shift;
			vec3 norm(0.0, 0.0, 1.0);
			Vertex v(pos, vec2(), norm);
			ret.vertexes.push_back(v);
		}
	}

	for (int i = 0; i < nx - 1; i++) {
		for (int j = 0; j < ny - 1; j++) {
			uvec3 idx;

			idx[0] = j + ny * i;
			idx[1] = j + (ny + 1) * i;
			idx[2] = j + (ny + 1) * i + 1;
			ret.faces.push_back(idx);

			idx[1] = idx[2];
			idx[2] = idx[0] + 1;
			ret.faces.push_back(idx);
		}
	}

	return ret;
}
