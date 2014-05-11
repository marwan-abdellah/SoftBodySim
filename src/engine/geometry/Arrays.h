#ifndef ARRAYS_H
#define ARRAYS_H

#include <glm/glm.hpp>
#include <vector>

/**
 * @brief Single vertex in mesh having its position, texture coords and normal.
 */
struct Vertex {
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

typedef std::vector<glm::vec3>                  vec3Array_t;
typedef std::vector<glm::vec2>                  vec2Array_t;
typedef std::vector<glm::float_t>               floatArray_t;
typedef std::vector<glm::uint>                  indexArray_t;
typedef std::vector<glm::uvec3>                 index3Array_t;
typedef std::vector<glm::uvec2>                 index2Array_t;
typedef std::vector<glm::uvec3>                 index4Array_t;
typedef std::vector<Vertex>                     vertexArray_t;

#endif
