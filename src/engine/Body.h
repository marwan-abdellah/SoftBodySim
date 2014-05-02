#ifndef SB_BODY_H
#define SB_BODY_H

#include "geometry/Sphere.h"

/**
 * @struct Mesh
 * @brief 
 *
 * @var Mesh::vertexes
 * @var Mesh::ElementBuffer
 */
struct Mesh {
	VertexBuffer                 *vertexes;
	ElementBuffer                *faces;
};

class Body {
public:
	virtual const Mesh *getMesh(void) = 0;
	virtual const Sphere *getBoundingVolume(void) = 0;
};

#endif
