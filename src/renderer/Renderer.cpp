#include "Renderer.h"
#include "common.h"
#include <stdio.h>
#include <GL/glu.h>

#include <glm/ext.hpp>

using namespace glm;

static const char *vertex_source = 
    "#version 330\n"
    "uniform mat4 projMatrix;"
    "uniform mat4 cameraMatrix;"
    "uniform vec3 color;"
    "in vec4 position;"
    "out vec4 fcolor;"

    "void main()"
    "{"
    "    gl_Position = projMatrix * cameraMatrix * position;"
    "    fcolor = vec4(color, 1);"
    "}";

static const char *fragment_source = 
    "#version 330\n"
    "in vec4 fcolor;"
    "void main(void) {"
    "    gl_FragColor = fcolor;"
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
    "uniform mat4 cameraMatrix;"
    "in vec2 fuv[];"
    "out vec2 muv;"
    "out vec3 normal;"
    "out vec4 position;"
    "void main() {"
    "    for(int i = 0; i < gl_in.length(); i++)"
    "    {"
    "        vec3 d1 = gl_in[0].gl_Position.xyz - gl_in[1].gl_Position.xyz;"
    "        vec3 d2 = gl_in[0].gl_Position.xyz - gl_in[2].gl_Position.xyz;"
    "        normal = normalize(cross(d1, d2));"
    "        gl_Position = projMatrix * cameraMatrix * gl_in[i].gl_Position;"
	"        position = projMatrix * gl_in[i].gl_Position;"
	"        muv = fuv[i];"
    "        EmitVertex();"
    "    }"
    "    EndPrimitive();"
    "}";

static const char *fragment_source2 = 
    "#version 330\n"
    "uniform vec3 lightSrc;"
	"uniform sampler2D mytext;"
    "uniform vec3 color;"
    "in vec2 muv;"
    "in vec3 normal;"
    "in vec4 position;"
    "out vec3 raster;"
    "void main(void) {"
    "    vec3 diff = normalize(lightSrc - position.xyz);"
    "    float d = max(0, dot(diff, normal));"
    "    vec3 mcolor = vec3(color.xyz * ( 0.2 + d));"
	"    if (muv != vec2(0,0))"
	"    raster = texture(mytext, muv).rgb * (0.2 + d);"
	"    else"
	"    raster = mcolor;"
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

    mProjectionMat = perspectiveFov(60.0f,
            (float)mWidth, (float)mHeight, 1.0f, 100.0f);

    mPointLine.setShaderSource(GL_VERTEX_SHADER, vertex_source);
    mPointLine.setShaderSource(GL_FRAGMENT_SHADER, fragment_source);
    mPointLine.compileAndLink();
    mPointLine.useShader();
    mPointLine.setUniform("projMatrix", mProjectionMat);

    mLighting.setShaderSource(GL_VERTEX_SHADER, vertex_source2);
    mLighting.setShaderSource(GL_GEOMETRY_SHADER, geometry_shader2);
    mLighting.setShaderSource(GL_FRAGMENT_SHADER, fragment_source2);
    mLighting.compileAndLink();
    mLighting.useShader();

    mLighting.setUniform("lightSrc", lightSrc);
    mLighting.setUniform("projMatrix", mProjectionMat);

    mCurrent = &mLighting;
}

void SoftBodyRenderer::shutdown()
{
}

void SoftBodyRenderer::clearScreen(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
}

//void SoftBodyRenderer::renderGround()
//{
//    const vec3 vColor(0.5, 0.5, 0.5);
//
//    if (!border)
//        return;
//
//    mCurrent->setUniform("color", &vColor);
//    mCurrent->setUniform("cameraMatrix", camMat);
//
//    border->draw();
//}

void SoftBodyRenderer::renderBody(Body *obj, const glm::mat4 &camMat)
{
    const VertexBuffer *buff;
    const ElementBuffer *ebuff;
	const MeshData *mesh;

    if (!obj)
        return;

    const vec3 &color = obj->GetColor();
	mesh = obj->GetMesh();

	if (mesh->material) {
		mesh->material->Bind();
	}
    mCurrent->setUniform("cameraMatrix", camMat);
    mCurrent->setUniform("color", color);

    buff = obj->GetVertexes();
    if (!buff) return;
    buff->Bind(VertexBuffer::VERTEX_ATTR_POSITION);

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
	if (mesh->material)
		mesh->material->Unbind();
}
