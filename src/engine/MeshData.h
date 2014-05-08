#ifndef SB_MESH_DATA_H
#define SB_MESH_DATA_H

#include "geometry/Arrays.h"
#include "geometry/Plane.h"
#include "geometry/Box.h"

#include <vector>

/**
 * @brief MeshData class 
 *
 * @brief Structure containing information about vertexes, normals, texture
 * coordinates of model mesh.
 */
class MeshData {
public:
	struct Vertex;
	typedef std::vector<Vertex> vertexArray_t;

	/**
	 * @brief Constructor
	 */
	MeshData(void) {}

	/**
	 * @brief Copy constructor
	 */
	MeshData(const MeshData &m) :
		vertexes(m.vertexes),
		faces(m.faces) {}

	/**
	 * @brief Creates plane mesh aligned to xy axis.
	 *
	 * @param[in] width width of plane
	 * @param[in] height height of plane
	 * @param[in] nx number of vertexes along x axis
	 * @param[in] ny number of vertexes along y axis
	 */
	static MeshData CreatePlane(float width, float height, size_t nx, size_t ny);

	/**
	 * @brief Creates cube mesh alighend to xyz axis.
	 *
	 * @param[in] box Box describing shape.
	 * @param[in] nx number of vertexes along x axis
	 * @param[in] ny number of vertexes along y axis
	 * @param[in] nz number of vertexes along z axis
	 */
	static MeshData CreateCube(const Box &box, size_t nx, size_t ny, size_t nz);

	/**
	 * @brief Creates mesh form Wavefron 'obj' text format.
	 *
	 * @param[in] path Wavefront file path.
	 */
	static MeshData CreateFromObj(const char *path);

	~MeshData(void) {}

	/**
	 * @brief Type of primitives to draw from vertexes.
	 */
	enum MeshType
	{
		POINTS,
		LINES,
		TRIANGLES
	};

	vertexArray_t  vertexes;
	index3Array_t  faces;
};

/**
 * @brief Single vertex in mesh having its position, texture coords and normal.
 */
struct MeshData::Vertex {
	/**
	 * @brief Constructor
	 *
	 * @param[in] v position of vertex in 3d space.
	 * @param[in] t texture coordinate of vertex.
	 * @param[in] n normal asociated with surface.
	 */
	Vertex(const glm::vec3 &v, const glm::vec2 &t, const glm::vec3 &n) :
		position(v),
		texture(t),
		normal(n) {}

	/**
	 * @brief Copy constructor
	 */
	Vertex(const Vertex &v) :
		position(v.position),
		texture(v.texture),
		normal(v.normal) {}

	~Vertex(void) {}

	glm::vec3 position;
	glm::vec2 texture;
	glm::vec3 normal;
};

#endif
