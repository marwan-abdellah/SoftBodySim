#include "SoftBody.h"
#include "common.h"
#include <cstring>

using namespace glm;

SoftBody::SoftBody(float_t mass, float_t springness, float_t damping,
         const vec3 *particles, unsigned int particles_count,
         const uvec2 *links_indexes, unsigned int links_count,
         const uvec4 *volumes_indexes, unsigned int volumes_count,
         const glm::uvec2 *text_coords, unsigned int text_coords_count,
         const glm::vec3 *normals, unsigned int normals_count,
         const glm::uvec3 *mesh_faces, unsigned int faces_count,
         VertexBuffer::VertexBufferType type)
{
    mMassInv = 1.0/mass;
    mSpringiness = springness;
    mDamping = damping;
    mParticles.resize(particles_count);
    mVelocities.resize(particles_count);
    mForces.resize(particles_count);
    mLinks.resize(links_count);
    mVolumes.resize(volumes_count);

    for(unsigned int i = 0; i < particles_count; i++)
        mParticles[i] = particles[i];
    for(unsigned int i = 0; i < links_count; i++) {
        LinkConstraint lnk;
        lnk.index = links_indexes[i];
        lnk.restLength = length(mParticles[lnk.index[0]] - mParticles[lnk.index[1]]);
        mLinks[i] = lnk;
    }
    // FIXME add volume constraint

    // prapare drawing data
    // Beacause vertexes can have different texture coordinates, normals 
    // depending on which face they belong, new vertices list has to be
    // created.
    vec3Array_t vertexes2, normal2;
    vec2Array_t textCoords2;
    index2Array_t edges2;
    index3Array_t faces2;

    for (unsigned int i = 0; i < faces_count; i++) {
    }
    switch (type) {
        case VertexBuffer::CPU_BUFFER:
            break;
        case VertexBuffer::OPENGL_BUFFER:
            mMesh = createGLVertexBufferMesh(&vertexes2, &textCoords2, &normal2, &edges2, &faces2);
            break;
    }
}

SoftBody::~SoftBody(void)
{
    if (!mMesh) return;
    if (mMesh->vertexes) delete mMesh->vertexes;
    if (mMesh->faces) delete mMesh->faces;
    if (mMesh->edges) delete mMesh->edges;
    delete mMesh;
}

Mesh_t *SoftBody::createGLVertexBufferMesh(vec3Array_t *vertexes, vec2Array_t *texCoords, vec3Array_t *normals,
        index2Array_t *edges, index3Array_t *faces)
{
    Mesh_t *mesh = new Mesh_t;
    GLVertexBuffer *buf;
    GLElementBuffer *ebuf;
    if (!mesh) return NULL;

    buf = new GLVertexBuffer(vertexes->size());
    if (!buf) {
        ERR("VertexBuffer memory allocation failed");
        delete mesh;
        return NULL;
    }
    buf->setVertexes(&vertexes->at(0));
    if (texCoords)
        buf->setTextCoords(&texCoords->at(0));
    if (normals)
        buf->setNormals(&normals->at(0));
    mesh->vertexes = buf;

    ebuf = new GLElementBuffer(edges->size(), ElementBuffer::EDGES);
    if (!ebuf) {
        ERR("ElementBuffer memory allocation failed");
        delete mesh->vertexes;
        delete mesh;
        return NULL;
    }
    ebuf->setIndexes2(&edges->at(0));
    mesh->edges = ebuf;

    ebuf = new GLElementBuffer(faces->size(), ElementBuffer::TRIANGLES);
    if (!ebuf) {
        ERR("ElementBuffer memory allocation failed");
        delete mesh->edges;
        delete mesh->vertexes;
        delete mesh;
        return NULL;
    }
    ebuf->setIndexes3(&faces->at(0));
    mesh->faces = ebuf;

    return mesh;
}
