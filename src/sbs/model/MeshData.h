#ifndef SB_MESH_DATA_H
#define SB_MESH_DATA_H

#include "sbs/geometry/Arrays.h"
#include "sbs/model/Material.h"
#include "sbs/model/OBJLexer.h"

#include <vector>
#include <set>
#include <unordered_map>

class Index3Hasher;
typedef std::unordered_map<glm::uvec3, unsigned int, Index3Hasher> vertex3Map_t;

/**
 * @brief MeshData class 
 *
 * @brief Class containing information about model mesh: vertexes, normals,
 * texture and indexes creating faces.
 * Class also collects additional information required to construct 
 * paticle systems like unique node positions 
 */
class MeshData {
public:
	typedef std::vector< std::set<int> > neighboursArray_t;


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
	const vec3Array_t &GetNodes(void) { return nodes; }

	/**
	 * @brief Connections between nodes.
	 */
	const index2Array_t &GetNodesConnections(void) { return nodesLinks; }

	/**
	 * @brief Indexes of nodes creating a mesh triangle.
	 *
	 * Might differ from faces array when vertexes and nodes differ.
	 */
	const index3Array_t &GetNodesTriangles(void) { return nodesTriangles; }

	// vertexes
	const vec3Array_t &GetVertexes(void) { return vertexes; }

	const vec3Array_t &GetNormals(void) { return normals; }

	const vec2Array_t &GetTextureCoords(void) { return textureCoords; }

	const indexArray_t &GetNodesLayout(void) { return vertexesNodes; }

	const index3Array_t &GetFaces(void) { return faces; }

	const index2Array_t &GetLines(void) { return edges; }

	void SetMaterial(const Material &m) { material = &m; }
	const Material *GetMaterial(void) const { return material; }

	const neighboursArray_t &GetNeighboursArray(void);
private:
	/**
	 * @brief Do not create empty mesh
	 */
	MeshData(void) {}

	/**
	 * @brief Do not copy mesh around
	 */
	MeshData(const MeshData &m) {}

	vec3Array_t nodes;
	index2Array_t nodesLinks;
	index3Array_t nodesTriangles;
	vec3Array_t    vertexes;
	indexArray_t   vertexesNodes;
	vec3Array_t    normals;
	vec2Array_t    textureCoords;
	index2Array_t  edges;
	index3Array_t  faces;

	const Material       *material;

	/**
	 * @brief Generates nodesTriangles data from faces.
	 */
	void GenerateTriangles(void);
	neighboursArray_t neighbours;

	static bool ProcessFace(OBJLexer &lexer, vertex3Map_t &map, vec2Array_t &textures, vec3Array_t &normals, MeshData *md);
};

#endif
