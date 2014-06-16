#ifndef SB_MESH_DATA_H
#define SB_MESH_DATA_H

#include "engine/geometry/Arrays.h"
#include "engine/model/Material.h"

#include <vector>
#include <set>

/**
 * @brief MeshData class 
 *
 * @brief Class containing information about model mesh: vertexes, normals,
 * texture and indexes creating faces.
 */
class MeshData {
public:
	typedef std::vector< std::set<int> > neighboursArray_t;

	/**
	 * @brief Constructor
	 */
	MeshData(void) : material(0) {}

	/**
	 * @brief Copy constructor
	 */
	MeshData(const MeshData &m) :
		nodes(m.nodes),
		nodesLinks(m.nodesLinks),
		vertexes(m.vertexes),
		vertexesNodes(m.vertexesNodes),
		faces(m.faces),
		material(m.material) {}

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
	 * @brief Creates sphere mesh
	 *
	 * @param[in] center Sphere center
	 * @param[in] radius Sphere radius
	 * @param[in] nv number of vertical lines
	 * @param[in] nh number of horizontal lines
	 */
	static MeshData *CreateSphere(glm::vec3 center, glm::float_t radius, size_t nv, size_t nh);

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
	 */
	index3Array_t nodesTriangles;

	// vertexes
	vec3Array_t    vertexes;
	vec3Array_t    normals;
	vec2Array_t    textureCoords;
	indexArray_t   vertexesNodes;

	index3Array_t  faces;
	index2Array_t  edges;
	Material       *material;

	const neighboursArray_t &GetNeighboursArray(void);
private:
	/**
	 * @brief Generates nodesTriangles data from faces.
	 */
	void GenerateTriangles(void);
	neighboursArray_t neighbours;
};

#endif
