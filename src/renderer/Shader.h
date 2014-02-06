#ifndef __SHADER_H
#define __SHADER_H

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

class Shader {
public:
	Shader(void) {}
	~Shader(void);
	void setShaderSource(GLenum type, const char *src);
	bool compileAndLink(void);
	void useShader(void);
	void setUniform(const char *name, const glm::vec3 *v);
	void setUniform(const char *name, const glm::mat4 *v);
private:
	GLuint createShader(GLenum, const char *src);
	const char *pVertexSrc;
	const char *pGeomSrc;
	const char *pFragSrc;
	GLuint iFragmentShader;
	GLuint iGeomShader;
	GLuint iVertexShader;
	GLuint iProgram;
};


#endif
