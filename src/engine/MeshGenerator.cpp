#include "MeshGenerator.h"
#include "common.h"
#include <map>

using namespace glm;

class uindx3Comp {
public:
	uindx3Comp(size_t y, size_t z) : m_y(y), m_z(z) {}
	uindx3Comp(const uindx3Comp &c) : m_y(c.m_y), m_z(c.m_z) {}
	bool operator()(const glm::uvec3 &a, const glm::uvec3 &b) { return a[0] * m_y * m_z + a[1] * m_z + a[2] > b[0] * m_y * m_z + b[1] * m_z + b[2] ; }
private:
	size_t m_y, m_z;
};

MeshData *MeshGenerator::generateFromCube(const Cube &c, size_t nx, size_t ny, size_t nz)
{
	if (nx < 2 || ny < 2 || nz < 2) {
		ERR("Invalid dimension");
		return NULL;
	}
	MeshData *ret = new MeshData;
	if (!ret) {
		ERR("new failed at %ld bytes", sizeof(MeshData));
		return NULL;
	}

	index3Array_t grid;
	uindx3Comp comp(ny, nz);
	std::map<glm::uvec3, glm::vec3, uindx3Comp> vertexMap(comp);

	glm::vec3 diff = c.m_upperRight - c.m_bottomLeft;

	glm::vec3 xdiff = 1.0f / (nx - 1) * glm::vec3(diff[0], 0, 0);
	glm::vec3 ydiff = 1.0f / (ny - 1) * glm::vec3(0, diff[1], 0);
	glm::vec3 zdiff = 1.0f / (nz - 1) * glm::vec3(0, 0, diff[2]);

#define ADD_FACE_VERTEXES(xs, xe, ys, ye, zs, ze) \
	grid.clear(); \
	for (unsigned int x = xs; x < xe; x++) \
		for (unsigned int y = ys; y < ye; y++) \
			for (unsigned int z = zs; z < ze; z++) {\
				glm::uvec3 ids(x, y, z);\
				glm::vec3 v = (float_t)x * xdiff + (float_t)y * ydiff + (float_t)z * zdiff;\
				v += c.m_bottomLeft; \
				vertexMap.insert( std::pair<glm::uvec3, glm::vec3>(ids, v) ); \
				grid.push_back(ids); \
			}\

#define ADD_FACE(a, b, c) {\
	glm::uvec3 vid1 = grid[a];\
	glm::uvec3 vid2 = grid[b];\
	glm::uvec3 vid3 = grid[c];\
	std::map<glm::uvec3, glm::vec3>::iterator it1 = vertexMap.find(vid1); \
	std::map<glm::uvec3, glm::vec3>::iterator it2 = vertexMap.find(vid2); \
	std::map<glm::uvec3, glm::vec3>::iterator it3 = vertexMap.find(vid3); \
	unsigned int d1 = std::distance(vertexMap.begin(), it1); \
	unsigned int d2 = std::distance(vertexMap.begin(), it2); \
	unsigned int d3 = std::distance(vertexMap.begin(), it3); \
	ret->faces.push_back(glm::uvec3(d1, d2, d3));\
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
		ret->vertexes.push_back(v);
	}
	return ret;
}
