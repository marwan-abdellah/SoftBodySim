#include "MeshData.h"
#include "common.h"

#include <unordered_map>
#include <unordered_set>

using namespace glm;
using namespace std;

#define BUCKET_MAX (1 << 20)

class Index3Hasher {
public:
	size_t operator() (uvec3 const &v) const {
		return (v[0] + 1759 * v[1] + 43517 * v[2]) & (BUCKET_MAX - 1);
	}
};

class Index2Hasher {
public:
	size_t operator() (uvec2 const &v) const {
		return (v[0] + 1759 * v[1]) & (BUCKET_MAX - 1);
	}
};

typedef unordered_map<uvec3, unsigned int, Index3Hasher> vertexMap_t;
typedef unordered_set<uvec2, Index2Hasher> linksSet_t;


static inline unsigned int
_node_insert(vertexMap_t &map, vec3Array_t &nods, uvec3 &id, vec3 &p)
{
	vertexMap_t::iterator it = map.find(id);
	if (it == map.end()) {
		pair<vertexMap_t::iterator, bool> res = map.insert(make_pair(id, nods.size()));
		nods.push_back(p);
		it = res.first;
	}
	return it->second;
}

MeshData MeshData::CreateCube(vec3 bottomLeftFront, vec3 upperRightBack, size_t nx, size_t ny, size_t nz)
{
	SB_ASSERT(((nx > 1) && (ny > 1) && (ny > 1)));

	MeshData ret;

	vec3 diff = upperRightBack - bottomLeftFront;
	diff[0] *= 1.0f / (nx - 1);
	diff[1] *= 1.0f / (ny - 1);
	diff[2] *= 1.0f / (nz - 1);

	unsigned int base = 0;
	vertexMap_t map;
	uvec3 id;

	// left plane
	for (unsigned int y = 0; y < ny; y++)
		for (unsigned int z = 0; z < nz; z++) {
			uvec3 id(0, y, z);
			vec3 p = vec3(id[0] * diff[0], id[1] * diff[1], id[2] * diff[2]) + bottomLeftFront;
			unsigned int d = _node_insert(map, ret.nodes, id, p);
			ret.vertexes.push_back(Vertex(p, vec2(), vec3(-1, 0, 0)));
			ret.vertexesNodes.push_back(d);
		}

	for (unsigned int y = 0; y < ny - 1; y++)
		for (unsigned int z = 0; z < nz - 1; z++) {
			unsigned int d1 = z + y * nz;
			unsigned int d2 = z + (y + 1) * nz;
			unsigned int d3 = z + (y + 1) * nz + 1;
			unsigned int d4 = z + y * nz + 1;

			ret.faces.push_back(uvec3(d1, d2, d3));
			ret.faces.push_back(uvec3(d1, d3, d4));
		}
	base = ret.vertexes.size();

	// right plane
	for (unsigned int y = 0; y < ny; y++)
		for (unsigned int z = 0; z < nz; z++) {
			uvec3 id(nx - 1, y, z);
			vec3 p = vec3(id[0] * diff[0], id[1] * diff[1], id[2] * diff[2]) + bottomLeftFront;
			unsigned int d = _node_insert(map, ret.nodes, id, p);
			ret.vertexes.push_back(Vertex(p, vec2(), vec3(1, 0, 0)));
			ret.vertexesNodes.push_back(d);
		}

	for (unsigned int y = 0; y < ny - 1; y++)
		for (unsigned int z = 0; z < nz - 1; z++) {
			unsigned int d1 = z + y * nz + base;
			unsigned int d2 = z + (y + 1) * nz + base;
			unsigned int d3 = z + (y + 1) * nz + 1 + base;
			unsigned int d4 = z + y * nz + 1 + base;

			ret.faces.push_back(uvec3(d1, d3, d2));
			ret.faces.push_back(uvec3(d1, d4, d3));
		}
	base = ret.vertexes.size();

	// bottom plane
	for (unsigned int x = 0; x < nx; x++)
		for (unsigned int z = 0; z < nz; z++) {
			uvec3 id(x, 0, z);
			vec3 p = vec3(id[0] * diff[0], id[1] * diff[1], id[2] * diff[2]) + bottomLeftFront;
			unsigned int d = _node_insert(map, ret.nodes, id, p);
			ret.vertexes.push_back(Vertex(p, vec2(), vec3(0, -1, 0)));
			ret.vertexesNodes.push_back(d);
		}

	for (unsigned int x = 0; x < nx - 1; x++)
		for (unsigned int z = 0; z < nz - 1; z++) {
			unsigned int d1 = z + x * nz + base;
			unsigned int d2 = z + (x + 1) * nz + base;
			unsigned int d3 = z + (x + 1) * nz + 1 + base;
			unsigned int d4 = z + x * nz + 1 + base;

			ret.faces.push_back(uvec3(d3, d2, d1));
			ret.faces.push_back(uvec3(d4, d3, d1));
		}
	base = ret.vertexes.size();

	// top plane
	for (unsigned int x = 0; x < nx; x++)
		for (unsigned int z = 0; z < nz; z++) {
			uvec3 id(x, ny - 1, z);
			vec3 p = vec3(id[0] * diff[0], id[1] * diff[1], id[2] * diff[2]) + bottomLeftFront;
			unsigned int d = _node_insert(map, ret.nodes, id, p);
			ret.vertexes.push_back(Vertex(p, vec2(), vec3(0, 1, 0)));
			ret.vertexesNodes.push_back(d);
		}

	for (unsigned int x = 0; x < nx - 1; x++)
		for (unsigned int z = 0; z < nz - 1; z++) {
			unsigned int d1 = z + x * nz + base;
			unsigned int d2 = z + (x + 1) * nz + base;
			unsigned int d3 = z + (x + 1) * nz + 1 + base;
			unsigned int d4 = z + x * nz + 1 + base;

			ret.faces.push_back(uvec3(d1, d2, d3));
			ret.faces.push_back(uvec3(d1, d3, d4));
		}
	base = ret.vertexes.size();

	// front plane
	for (unsigned int x = 0; x < nx; x++)
		for (unsigned int y = 0; y < ny; y++) {
			uvec3 id(x, y, 0);
			vec3 p = vec3(id[0] * diff[0], id[1] * diff[1], id[2] * diff[2]) + bottomLeftFront;
			unsigned int d = _node_insert(map, ret.nodes, id, p);
			ret.vertexes.push_back(Vertex(p, vec2(), vec3(0, 0, 1)));
			ret.vertexesNodes.push_back(d);
		}

	for (unsigned int x = 0; x < nx - 1; x++)
		for (unsigned int y = 0; y < ny - 1; y++) {
			unsigned int d1 = y + x * ny + base;
			unsigned int d2 = y + (x + 1) * ny + base;
			unsigned int d3 = y + (x + 1) * ny + 1 + base;
			unsigned int d4 = y + x * ny + 1 + base;

			ret.faces.push_back(uvec3(d1, d2, d3));
			ret.faces.push_back(uvec3(d1, d3, d4));
		}
	base = ret.vertexes.size();

	// back plane
	for (unsigned int x = 0; x < nx; x++)
		for (unsigned int y = 0; y < ny; y++) {
			uvec3 id(x, y, nz - 1);
			vec3 p = vec3(id[0] * diff[0], id[1] * diff[1], id[2] * diff[2]) + bottomLeftFront;
			unsigned int d = _node_insert(map, ret.nodes, id, p);
			ret.vertexes.push_back(Vertex(p, vec2(), vec3(0, 0, -1)));
			ret.vertexesNodes.push_back(d);
		}

	for (unsigned int x = 0; x < nx - 1; x++)
		for (unsigned int y = 0; y < ny - 1; y++) {
			unsigned int d1 = y + x * ny + base;
			unsigned int d2 = y + (x + 1) * ny + base;
			unsigned int d3 = y + (x + 1) * ny + 1 + base;
			unsigned int d4 = y + x * ny + 1 + base;

			ret.faces.push_back(uvec3(d3, d2, d1));
			ret.faces.push_back(uvec3(d4, d3, d1));
		}

	ret.GenerateLinks();

	return ret; // by value - assume compiler RVO
}

MeshData MeshData::CreatePlane(float width, float height, size_t nx, size_t ny)
{
	SB_ASSERT((nx > 1) && (ny > 1) && (width > 0.0) && (height > 0.0));
	MeshData ret;

	const float xstep = width / (nx - 1);
	const float ystep = height / (ny - 1);

	vec3 shift(width / 2.0, height / 2.0, 0.0);

	for (unsigned int x = 0; x < nx; x++) {
		for (unsigned int y = 0; y < ny; y++) {
			vec3 pos(x * xstep, y * ystep, 0.0);
			pos = pos - shift;
			vec3 norm(0.0, 0.0, 1.0);
			Vertex v(pos, vec2(), norm);
			ret.vertexes.push_back(v);
		}
	}

	for (unsigned int x = 0; x < nx - 1; x++) {
		for (unsigned int y = 0; y < ny - 1; y++) {
			uvec3 idx;

			idx[0] = y + ny * x;
			idx[1] = y + (ny + 1) * x;
			idx[2] = y + (ny + 1) * x + 1;
			ret.faces.push_back(idx);

			idx[1] = idx[2];
			idx[2] = idx[0] + 1;
			ret.faces.push_back(idx);
		}
	}

	ret.GenerateLinks();

	return ret;
}

void MeshData::GenerateLinks(void)
{
	nodesLinks.clear();
	linksSet_t set;

	for (index3Array_t::iterator it = faces.begin(); it != faces.end(); it++) {
		unsigned int id1 = vertexesNodes[(*it)[0]];
		unsigned int id2 = vertexesNodes[(*it)[1]];
		unsigned int id3 = vertexesNodes[(*it)[2]];

		uvec2 ed1((id1 < id2 ? id1 : id2), (id2 > id1 ? id2 : id1));
		uvec2 ed2((id2 < id3 ? id2 : id3), (id3 > id2 ? id3 : id2));
		uvec2 ed3((id3 < id1 ? id3 : id1), (id1 > id3 ? id1 : id3));

		set.insert(ed1);
		set.insert(ed2);
		set.insert(ed3);

		edges.push_back(uvec2((*it)[0], (*it)[1]));
		edges.push_back(uvec2((*it)[1], (*it)[2]));
		edges.push_back(uvec2((*it)[2], (*it)[0]));
	}

	for (linksSet_t::iterator it = set.begin(); it != set.end(); it++)
		nodesLinks.push_back(*it);
}
