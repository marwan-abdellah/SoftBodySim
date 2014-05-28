#ifndef SB_MESH_DATA_H
#define SB_MESH_DATA_H

#include "engine/geometry/Arrays.h"
#include "engine/geometry/Plane.h"

#include <vector>

/**
 * @brief MeshData class 
 *
 * @brief Class containing information about model mesh: vertexes, normals,
 * texture and indexes creating faces.
 */
class MeshData {
public:
	struct Vertex;
	typedef std::vector<Vertex>                 vertexArray_t;
	typedef std::vector<Vertex>::iterator       vertexArrayIterator_t;
	typedef std::vector<Vertex>::const_iterator vertexArrayConstIterator_t;

	/**
	 * @brief Constructor
	 */
	MeshData(void) {}

	/**
	 * @brief Copy constructor
	 */
	MeshData(const MeshData &m) :
		nodes(m.nodes),
		nodesLinks(m.nodesLinks),
		vertexes(m.vertexes),
		vertexesNodes(m.vertexesNodes),
		faces(m.faces) {}

	/**
	 * @brief Creates plane mesh aligned to xy axis.
	 *
	 * @param[in] width width of plane
	 * @param[in] height height of plane
	 * @param[in] nx number of vertexes along x axis
	 * @param[in] ny number of vertexes along y axis
	 */
	static MeshData *CreatePlane(float width, float height, size_t nx, size_t ny);

	/**
	 * @brief Creates cube mesh alighend to xyz axis.
	 *
	 * @param[in] leftLowerFront Cube left lower front corner.
	 * @param[in] rightUpperBack Cube right upper back corner.
	 * @param[in] nx number of vertexes along x axis
	 * @param[in] ny number of vertexes along y axis
	 * @param[in] nz number of vertexes along z axis
	 */
	static MeshData *CreateCube(glm::vec3 leftLowerFront, glm::vec3 rightUpperBack, size_t nx, size_t ny, size_t nz);

	/**
	 * @brief Creates mesh form Wavefron 'obj' text format.
	 *
	 * @param[in] path Wavefront file path.
	 */
	static MeshData *CreateFromObj(const char *path);

	~MeshData(void) {}

	/**
	 * @brief Generates nodesLinks data from faces.
	 */
	void GenerateLinks(void);

	/**
	 * @brief Type of primitives to draw from vertexes.
	 */
	enum MeshType
	{
		POINTS,
		LINES,
		TRIANGLES
	};

	/**
	 * @brief Nodes of 3D model mesh. Each node can map to multiple 
	 * vertexes depending on normal and texture coordinates attached to
	 * vertex.
	 */
	vec3Array_t nodes;

	/**
	 * @brief Connections between nodes.
	 */
	index2Array_t nodesLinks;

	/**
	 * @brief Indexes of nodes creating a mesh triangle.
	 *
	 * Might differ from faces array when vertexes and nodes differ.
	 * FIXME can be sharead accross meshes of same type
	 */
	index3Array_t nodesTriangles;

	vertexArray_t  vertexes;
	indexArray_t   vertexesNodes;

	index3Array_t  faces;
	index2Array_t  edges;
private:
	/**
	 * @brief Generates nodesTriangles data from faces.
	 */
	void GenerateTriangles(void);
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
		normal(n)
		{}

	/**
	 * @brief Copy constructor
	 */
	Vertex(const Vertex &v) :
		position(v.position),
		texture(v.texture),
		normal(v.normal)
		{}

	~Vertex(void) {}

	glm::vec3 position;
	glm::vec2 texture;
	glm::vec3 normal;
};


#endif
