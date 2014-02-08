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

SoftBody::SoftBody(glm::float_t mass, glm::float_t springness, glm::float_t damping,
             vec3Array_t *particles, index2Array_t *links, index4Array_t *volumes,
             vec2Array_t *textCoords, facesArray_t *faces,
             VertexBuffer::VertexBufferType type)
{
    mMassInv = 1.0/mass;
    mSpringiness = springness;
    mDamping = damping;
    mParticles = *particles;
    mVelocities.resize(mParticles.size());
    mForces.resize(mParticles.size());
    mLinks.resize(links->size());
    mVolumes.resize(volumes->size());

    for(unsigned int i = 0; i < links->size(); i++) {
        LinkConstraint lnk;
        lnk.index = links->at(i);
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
    // f: 1/1 2  # if there is no texture id assume it is 0
    cache_t *cache = cacheInit(mParticles.size());

    for (unsigned int i = 0; i < faces->size(); i++) {
        unsigned int faceSize = faces->at(i).size();
        if (faceSize != 3 && faceSize != 2) {
            ERR("Invalid face size. Expected 2 or 3 vertices faces only! Ignoring face: %d", i + 1);
            continue;
        }
        uvec3 vids(0, 0, 0);
        for (unsigned int j = 0; j < faceSize; j++) {
            unsigned int vid = faces->at(i)[j][0];
            unsigned int tid = faces->at(i)[j][1];
            int idx = cacheGetValue(cache, vid, tid);
            if (idx == -1) {
                // add pair (vertex id/texture id) to cache
                vertexes2.push_back(mParticles[vid - 1]);
                mMeshVertexParticleMapping.push_back(vid - 1);
                // add texture only if texture id != 0
                if (tid) textCoord2.push_back(textCoords->at(tid - 1));
                idx = vertexes2.size() - 1;
                cacheAddValue(cache, vid, tid, idx);
            }
            vids[j] = idx;
        }
        edges2.push_back(uvec2(vids[0], vids[1]));
        if (faceSize == 3) {
            edges2.push_back(uvec2(vids[1], vids[2]));
            edges2.push_back(uvec2(vids[2], vids[0]));
            faces2.push_back(uvec3(vids[0], vids[1], vids[2]));
        }
    }
    delete cache;

    // some extra checks
    if (vertexes2.size() != textCoord2.size()) {
        ERR("Something goes bad. |Vertex| != |TextCoords|. Aborting buffer creation.");
        return;
    }

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
    Mesh_t *mesh;
    GLVertexBuffer *buf;
    GLElementBuffer *ebuf;

    if (!vertexes || vertexes->size() == 0) return NULL;

    mesh = new Mesh_t;
    if (!mesh) return NULL;
    memset(mesh, 0x0, sizeof(Mesh_t));

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

    if (edges && edges->size() > 0) {
        ebuf = new GLElementBuffer(edges->size(), ElementBuffer::EDGES);
        if (!ebuf) {
            ERR("ElementBuffer memory allocation failed");
            delete mesh->vertexes;
            delete mesh;
            return NULL;
        }
        ebuf->setIndexes2(&edges->at(0));
        mesh->edges = ebuf;
    }

    if (faces && faces->size() > 0) {
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
    }

    return mesh;
}
