#ifndef SB_MESH_GENERATOR_H
#define SB_MESH_GENERATOR_H

#include "utils/geometry/MeshData.h"

class MeshGenerator
{
public:
	static MeshData *generateFromCube(const Cube &c, size_t nx, size_t ny, size_t nz);
private:
	MeshGenerator(void) {}
	MeshGenerator(const MeshGenerator&) {}
	~MeshGenerator(void) {}
};

#endif
