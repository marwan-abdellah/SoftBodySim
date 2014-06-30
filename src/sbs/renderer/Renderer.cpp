#include "common.h"

#include <stdio.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>

#include "Renderer.h"

#define SPHERE_RADIUS 2.0f

using namespace glm;

static const char *vertex_source = 
    "#version 330\n"
    "uniform mat4 projMatrix;"
    "uniform mat4 modeViewMatrix;"
    "in vec4 position;"

    "void main()"
    "{"
    "    gl_Position = projMatrix * modeViewMatrix * position;"
    "}";

static const char *fragment_source = 
    "#version 330\n"
    "out vec3 color;"
	"uniform vec3 incolor;"
    "void main(void) {"
    "    color = incolor;"
    "}";

static const char *vertex_source2 = 
    "#version 330\n"
    "in vec3 position;"
    "in vec2 texture;"
    "out vec2 fuv;"
    "void main()"
    "{"
    "    gl_Position = vec4(position,1);"
	"    fuv = texture;"
    "}";

static const char *geometry_shader2 = 
    "#version 330\n"
    "layout (triangles) in;"
    "layout (triangle_strip, max_vertices = 3) out;"
    "uniform mat4 projMatrix;"
    "uniform mat4 modeViewMatrix;"
    "in vec2 fuv[];"
    "out vec2 muv;"
    "out vec3 normal;"
    "out vec4 position;"
    "void main() {"
    "    vec3 d1 = gl_in[0].gl_Position.xyz - gl_in[1].gl_Position.xyz;"
    "    vec3 d2 = gl_in[0].gl_Position.xyz - gl_in[2].gl_Position.xyz;"
    "    normal = normalize(cross(d1, d2));"
    "    for(int i = 0; i < gl_in.length(); i++)"
    "    {"
    "        gl_Position = projMatrix * modeViewMatrix * gl_in[i].gl_Position;"
	"        position = gl_in[i].gl_Position;"
	"        muv = fuv[i];"
    "        EmitVertex();"
    "    }"
    "    EndPrimitive();"
    "}";

static const char *fragment_source2 = 
    "#version 330\n"
    "uniform vec3 lightSrc;"
	"uniform sampler2D mytext;"
    "in vec2 muv;"
    "in vec3 normal;"
    "in vec4 position;"
    "out vec3 color;"
    "void main(void) {"
    "    vec3 diff = normalize(lightSrc - position.xyz);"
    "    float d = max(0, dot(diff, normal));"
	"    color = texture(mytext, muv).rgb * (0.2 + d);"
    "}";

void SoftBodyRenderer::setRenderMethod(SoftBodyRenderMethod_e m)
{
    if (m == SB_RENDER_FACES) {
        mLighting.useShader();
        mCurrent = &mLighting;
    }
    if (m == SB_RENDER_PARTICLES) {
        mPointLine.useShader();
        mCurrent = &mPointLine;
    }
    mMethod = m;
}

SoftBodyRenderMethod_e    SoftBodyRenderer::getRenderMethod(void)
{
    return mMethod;
}

void SoftBodyRenderer::logShader(GLint shader)
{
    GLint len;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
    char *log = new char[len];
    glGetShaderInfoLog(shader, len, &len, log);
    ERR("Shader log: %s", log);
    delete log;
}

void SoftBodyRenderer::initialize(int width, int height)
{
    mWidth = width;
    mHeight = height;
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    mMethod = SB_RENDER_FACES;

    vec3 lightSrc(-5, 1, 0);

    mProjectionMat = perspectiveFov(pi<float_t>() / 3.0f,
            (float)mWidth, (float)mHeight, 1.0f, 100.0f);

    mPointLine.setShaderSource(GL_VERTEX_SHADER, vertex_source);
    mPointLine.setShaderSource(GL_FRAGMENT_SHADER, fragment_source);
    mPointLine.compileAndLink();
    mPointLine.useShader();
    mPointLine.setUniform("projMatrix", mProjectionMat);
	mPointLine.setUniform("incolor", vec3(0.0, 0.0, 1.0));

    mLighting.setShaderSource(GL_VERTEX_SHADER, vertex_source2);
    mLighting.setShaderSource(GL_GEOMETRY_SHADER, geometry_shader2);
    mLighting.setShaderSource(GL_FRAGMENT_SHADER, fragment_source2);
    mLighting.compileAndLink();
    mLighting.useShader();

    mLighting.setUniform("lightSrc", lightSrc);
    mLighting.setUniform("projMatrix", mProjectionMat);

	MeshData *mesh = MeshData::CreateSphere(vec3(0,0,0), SPHERE_RADIUS, 20, 20);
	mSphere = new VertexBuffer(mesh->GetVertexes().size(), VertexBuffer::STATIC);
	mSphere->SetVertexes(mesh->GetVertexes());
	delete mesh;

    mCurrent = &mLighting;
}

void SoftBodyRenderer::shutdown()
{
}

SoftBodyRenderer::SoftBodyRenderer() :
	mWorld(0)
{
}

void SoftBodyRenderer::DrawWorld(const mat4 &cam)
{
	if (!mWorld) return;
	mPointLine.useShader();
	mPointLine.setUniform("incolor", vec3(0.4, 0.4, 0.4));
    mPointLine.setUniform("modeViewMatrix", cam);
	mWorld->Draw(QUADS);
	mPointLine.setUniform("incolor", vec3(0.0, 0.0, 1.0));
	mCurrent->useShader();
}

void SoftBodyRenderer::SetWorld(SoftBodySolver::SoftBodyWorldParameters &p)
{
	if (mWorld) delete mWorld;

	vec3Array_t vb;
	float_t groundLevel = p.groundLevel - 0.001f; // avoid z-fighting
	vb.push_back(vec3(p.rightWall, groundLevel, p.frontWall));
	vb.push_back(vec3(p.rightWall, groundLevel, p.backWall));
	vb.push_back(vec3(p.leftWall, groundLevel, p.backWall));
	vb.push_back(vec3(p.leftWall, groundLevel, p.frontWall));

	mWorld = new VertexBuffer(vb.size(), VertexBuffer::STATIC);
	mWorld->SetVertexes(vb);
}

void SoftBodyRenderer::clearScreen(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
}

void SoftBodyRenderer::renderBody(Body *obj, const glm::mat4 &camMat)
{
    const VertexBuffer *buff;
    const ElementBuffer *ebuff;
	const MeshData *mesh;
	mat4 modelView = camMat * obj->GetModelMatrix();
	const Sphere &bs = obj->GetBoundingSphere();
	const Material *material;

	float_t fac = bs.mRadius / SPHERE_RADIUS;
	mat4 camTrans = translate(camMat, bs.mCenter);
	camTrans = scale(camTrans, vec3(fac, fac, fac));
	mPointLine.useShader();
    mPointLine.setUniform("modeViewMatrix", camTrans);
	mSphere->Bind(0);
	mSphere->Draw(POINTS);
	mSphere->Unbind();

	mCurrent->useShader();
	mesh = obj->GetMesh();
	material = mesh->GetMaterial();
	if (material) material->Bind();

    mCurrent->setUniform("modeViewMatrix", modelView);

    buff = obj->GetVertexes();
    if (!buff) return;
    buff->Bind(0);

    switch (mMethod) {
        case SB_RENDER_PARTICLES:
            ebuff = obj->getEdges();
			break;
    case SB_RENDER_FACES:
            ebuff = obj->getFaces();
			break;
    }
	ebuff->Draw();
    buff->Unbind();
	if (material) material->Unbind();
}
