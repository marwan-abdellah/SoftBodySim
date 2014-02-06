#include "Renderer.h"
#include <stdio.h>
#include <GL/glu.h>

#include <glm/ext.hpp>

using namespace glm;

#define ERR(x, ...)

#define WIN_HEIGHT 1024
#define WIN_WIDTH 768

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
    "     fcolor = vec4(color, 1);"
    "};";

static const char *fragment_source = 
    "#version 330\n"
    "in vec4 fcolor;"
    "void main(void) {"
    "    gl_FragColor = fcolor;"
      "}";

static const char *vertex_source2 = 
    "#version 330\n"
    "uniform vec3 color;"
    "in vec4 position;"
    "out vec4 fcolor;"
    "void main()"
    "{"
    "    gl_Position = position;"
    "     fcolor = vec4(color, 1);"
    "}";

static const char *geometry_shader2 = 
    "#version 330\n"
    "layout (triangles) in;"
    "layout (triangle_strip, max_vertices = 3) out;"
    "in vec4 fcolor[];"
    "uniform mat4 projMatrix;"
    "uniform mat4 cameraMatrix;"
    "uniform vec3 lightSrc;"
    "out vec4 mcolor;"
    "void main() {"
    "    for(int i = 0; i < gl_in.length(); i++)"
    "    {"
    "        vec3 d1 = gl_in[0].gl_Position.xyz - gl_in[1].gl_Position.xyz;"
    "        vec3 d2 = gl_in[0].gl_Position.xyz - gl_in[2].gl_Position.xyz;"
    "        vec3 norm = normalize(cross(d1, d2));"
    "         vec3 diff = normalize(lightSrc - gl_in[i].gl_Position.xyz);"
    "       float d = max(0, dot(diff, norm));"
    "        gl_Position = projMatrix * cameraMatrix * gl_in[i].gl_Position;"
    "        mcolor = vec4(fcolor[i].xyz * ( 0.2 + d), 1);"
    "        EmitVertex();"
    "    }"
    "    EndPrimitive();"
    "}"
    "";

static const char *fragment_source2 = 
    "#version 330\n"
    "in vec4 mcolor;"
    "void main(void) {"
    "    gl_FragColor = mcolor;"
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

void SoftBodyRenderer::initialize(int width = WIN_HEIGHT, int height = WIN_WIDTH)
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

    mProjectionMat = mProjectionMat * perspectiveFov(60.0f,
            (float)mHeight, (float)mWidth, 1.0f, 100.0f);

    mPointLine.setShaderSource(GL_VERTEX_SHADER, vertex_source);
    mPointLine.setShaderSource(GL_FRAGMENT_SHADER, fragment_source);
    mPointLine.compileAndLink();
    mPointLine.useShader();
    mPointLine.setUniform("projMatrix", &mProjectionMat);

    mLighting.setShaderSource(GL_VERTEX_SHADER, vertex_source2);
    mLighting.setShaderSource(GL_FRAGMENT_SHADER, fragment_source2);
    mLighting.setShaderSource(GL_GEOMETRY_SHADER, geometry_shader2);
    mLighting.compileAndLink();
    mLighting.useShader();
    mLighting.setUniform("lightSrc", &lightSrc);
    mLighting.setUniform("projMatrix", &mProjectionMat);

    mCurrent = &mLighting;
}

void SoftBodyRenderer::shutdown()
{
}

void SoftBodyRenderer::clearScreen(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
}

//void SoftBodyRenderer::RenderBorder(PlainBorder *border, const glm::mat4 *camMat)
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

void SoftBodyRenderer::renderBody(SoftBody *obj, const glm::mat4 *camMat)
{
//    vec3 color;
//
//    if (!obj)
//        return;
//
//    mCurrent->setUniform("cameraMatrix", camMat);
//
//    color = obj->getColor();
//    mCurrent->setUniform("color", &color);
//
//    SoftBody::DrawMethod m;
//
//    switch (mMethod) {
//        case SB_RENDER_PARTICLES:
//            m = SoftBody::CONNECTIONS;
//        break;
//    case SB_RENDER_FACES:
//            m = SoftBody::FACES;
//        break;
//    }
//    obj->setDrawMethod(m);
//    obj->draw();
}
