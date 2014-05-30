#include "Shader.h"
#include "common.h"

using namespace glm;

Shader::Shader(void) :
    pVertexSrc(0),
    pGeomSrc(0),
    pFragSrc(0)
{
}

Shader::~Shader(void) {
    if (iFragmentShader) glDeleteShader(iFragmentShader);
    if (iVertexShader) glDeleteShader(iVertexShader);
    if (iGeomShader) glDeleteShader(iGeomShader);
    if (iProgram) glDeleteProgram(iProgram);
}

void Shader::setShaderSource(GLenum type, const char *src)
{
    switch(type) {
        case GL_VERTEX_SHADER:
            pVertexSrc = src;
            break;
        case GL_FRAGMENT_SHADER:
            pFragSrc = src;
            break;
        case GL_GEOMETRY_SHADER:
            pGeomSrc = src;
            break;
        default:
            ERR("Invalid shader type.");
    }
}

GLuint Shader::createShader(GLenum type, const char *src)
{
    GLuint shader;
    GLint status, len;
    char *log;

    shader = glCreateShader(type);
    if (shader == 0) {
        ERR("glCreateShader failed.");
        return 0;
    }
    
    glShaderSource(shader, 1, (const GLchar**)&src, NULL);
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

    if (status != GL_TRUE) {
        switch(type) {
            case GL_VERTEX_SHADER:
                ERR("Vertex Shader compilation failed");
                break;
            case GL_FRAGMENT_SHADER:
                ERR("Fragment Shader compilation failed");
                break;
            case GL_GEOMETRY_SHADER:
                ERR("Geometry Shader compilation failed");
                break;
            default:
                break;
        }
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        log = new char[len];
        glGetShaderInfoLog(shader, len, &len, log);
        ERR("%s", log);
        delete log;
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

bool Shader::compileAndLink(void)
{
    GLint status;
    if (!pVertexSrc || !pFragSrc) {
        ERR("Vertex and Fragment shader are required.");
        return false;
    }
    iFragmentShader = createShader(GL_FRAGMENT_SHADER, pFragSrc);
    iVertexShader = createShader(GL_VERTEX_SHADER, pVertexSrc);

    if (!iVertexShader || !iFragmentShader)
        goto on_error;

    if (pGeomSrc) {
        iGeomShader = createShader(GL_GEOMETRY_SHADER, pGeomSrc);
        if (!iGeomShader)
            goto on_error;
    }

    iProgram = glCreateProgram();
    if (!iProgram)
        goto on_error;

    glAttachShader(iProgram, iVertexShader);
    glAttachShader(iProgram, iFragmentShader);
    if (iGeomShader)
        glAttachShader(iProgram, iGeomShader);

    glLinkProgram(iProgram);

    glGetProgramiv(iProgram, GL_LINK_STATUS, &status);
    if (status != GL_TRUE) {
        ERR("glLinkProgram failed.");
        goto on_error;
    }
    DBG("Shader compile & link successed.");
    return true;

on_error:
    if (iFragmentShader) glDeleteShader(iFragmentShader);
    if (iVertexShader) glDeleteShader(iVertexShader);
    if (iGeomShader) glDeleteShader(iGeomShader);
    if (iProgram) glDeleteProgram(iProgram);
	int len = 0;
	glGetProgramiv(iProgram, GL_INFO_LOG_LENGTH, &len);
	if (len > 0) {
		char *log = new char[len];
		glGetProgramInfoLog(iProgram, len, &len, log);
		ERR("%s", log);
		delete log;
	}
	iProgram = 0;
    return false;
}

void Shader::useShader(void)
{
    if (iProgram)
        glUseProgram(iProgram);
    else
        ERR("Program failed to compile and link.");
}

void Shader::setUniform(const char *name, const glm::vec3 &v)
{
    if (!iProgram)
        return;
    GLint k = glGetUniformLocation(iProgram, name);
    if (k == -1) {
        ERR("Invalid uniform name: %s", name);
        return;
    }
    glUniform3fv(k, 1, glm::value_ptr(v));
}

void Shader::setUniform(const char *name, const glm::mat4 &v)
{
    if (!iProgram)
        return;
    GLint k = glGetUniformLocation(iProgram, name);
    if (k == -1) {
        ERR("Invalid uniform name: %s", name);
        return;
    }
    glUniformMatrix4fv(k, 1, false, glm::value_ptr(v));
}
