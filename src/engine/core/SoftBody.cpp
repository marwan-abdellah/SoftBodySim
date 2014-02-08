#include "SoftBody.h"
#include "common.h"
#include <cstring>

using namespace glm;
using namespace std;

typedef struct cache_item {
    unsigned int minor;
    int value;
} cache_item_t;

typedef vector< vector<cache_item_t> > cache_t;

static cache_t *cacheInit(unsigned int size)
{
    return new cache_t(size);
}

static int cacheGetValue(cache_t *cache, unsigned int major, unsigned int minor)
{
    FOREACH(it, &(cache->at(major))) {
        if (it->minor == minor)
            return it->value;
    }
    return -1;
}
static void cacheAddValue(cache_t *cache, unsigned int major, unsigned int minor, unsigned int value)
{
    cache_item_t item;
    item.minor = minor;
    item.value = value;

    cache->at(major).push_back(item);
}

SoftBody::SoftBody(float_t mass, float_t springness, float_t damping,
         const vec3 *particles, unsigned int particles_count,
         const uvec2 *links_indexes, unsigned int links_count,
         const uvec4 *volumes_indexes, unsigned int volumes_count,
         const glm::vec2 *tex_coords, unsigned int text_coords_count,
         const glm::uvec2 *mesh_faces, unsigned int faces_count,
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
    vec3Array_t vertexes2;
    vec2Array_t textCoord2;
    index2Array_t edges2;
    index3Array_t faces2;

    // mesh face can contain 2 or 3 groups of series index/textCoord/normal
    // f: 1/1 2/2 3/3
    // f: 1/1 2/2
    // f: 1/1 2/2  # if there is no 3rd verticle assume it is 0/0
    cache_t *cache = cacheInit(mParticles.size());
    for (unsigned int i = 0; i < faces_count; i+=3) {
        uvec2 v1 = mesh_faces[i];
        uvec2 v2 = mesh_faces[i + 1];
        uvec2 v3 = mesh_faces[i + 2];

        unsigned int vid1 = v1[0];
        unsigned int tid1 = v1[1];
        int idx1 = cacheGetValue(cache, vid1, tid1);
        if (idx1 == -1) {
            vertexes2.push_back(mParticles[vid1 - 1]);
            mMeshVertexParticleMapping.push_back(vid1 - 1);
            if (tid1) textCoord2.push_back(tex_coords[tid1 - 1]);
            idx1 = vertexes2.size() - 1;
            cacheAddValue(cache, vid1, tid1, idx1);
        }

        unsigned int vid2 = v2[0];
        unsigned int tid2 = v2[1];
        int idx2 = cacheGetValue(cache, vid2, tid2);
        if (idx2 == -1) {
            vertexes2.push_back(mParticles[vid2 - 1]);
            mMeshVertexParticleMapping.push_back(vid2 - 1);
            if (tid2) textCoord2.push_back(tex_coords[tid2 - 1]);
            idx2 = vertexes2.size() - 1;
            cacheAddValue(cache, vid2, tid2, idx2);
        }

        unsigned int vid3 = v3[0];
        unsigned int tid3 = v3[1];
        int idx3 = cacheGetValue(cache, vid3, tid3);
        if (idx3 == -1) {
            vertexes2.push_back(mParticles[vid3 - 1]);
            mMeshVertexParticleMapping.push_back(vid3 - 1);
            if (tid3) textCoord2.push_back(tex_coords[tid3 - 1]);
            idx3 = vertexes2.size() - 1;
            cacheAddValue(cache, vid3, tid3, idx3);
        }

        uvec2 edidx1(idx1, idx2);
        uvec2 edidx2(idx2, idx3);
        uvec2 edidx3(idx3, idx1);

        uvec3 tridx(idx1, idx2, idx3);

        edges2.push_back(edidx1);
        edges2.push_back(edidx2);
        edges2.push_back(edidx3);

        faces2.push_back(tridx);
    }
    delete cache;

    switch (type) {
        case VertexBuffer::CPU_BUFFER:
            break;
        case VertexBuffer::OPENGL_BUFFER:
            mMesh = createGLVertexBufferMesh(&vertexes2, &textCoord2, &edges2, &faces2);
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

Mesh_t *SoftBody::createGLVertexBufferMesh(vec3Array_t *vertexes, vec2Array_t *texCoords,
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
    if (texCoords && texCoords->size() > 0)
        buf->setTextCoords(&texCoords->at(0));
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
