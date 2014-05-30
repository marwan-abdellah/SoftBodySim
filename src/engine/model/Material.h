#ifndef MATERIAL_H_
#define MATERIAL_H_

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

class Material {
public:
	Material(void);
	~Material(void);
	int LoadTextureFromBmp(const char *path);
	void Bind(void);
	void Unbind(void);
private:
	GLuint mDiffuseTexture;
};

#endif
