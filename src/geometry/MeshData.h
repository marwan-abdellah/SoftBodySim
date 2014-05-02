#ifndef SB_MESH_DATA_H
#define SB_MESH_DATA_H

#include "geometry/Arrays.h"
#include "geometry/Plane.h"
#include "geometry/Cube.h"

#include <vector>

/**
 * @struct Vertex
 *
 * @brief 
 */
struct Vertex {
	Vertex(const Vertex &v) :
		position(v.position),
		texture(v.texture),
		normal(v.normal) {}
	Vertex(const glm::vec3 &v, const glm::vec2 &t, const glm::vec3 &n) :
		position(v),
		texture(t),
		normal(n) {}
	~Vertex(void) {}

	glm::vec3 position;
	glm::vec2 texture;
	glm::vec3 normal;
};

typedef std::vector<Vertex> vertexArray_t;

/**
 * @struct MeshData
 *
 * @brief Structure containing information about vertexes, normals, texture
 * coordinates of model mesh.
 */
struct MeshData {
	MeshData(void) {}
	MeshData(const MeshData &m) :
		vertexes(m.vertexes),
		faces(m.faces),
		links(m.links) {}
	~MeshData(void) {}

	vertexArray_t  vertexes;
	index3Array_t  faces;
	index2Array_t  links;
};

#endif
