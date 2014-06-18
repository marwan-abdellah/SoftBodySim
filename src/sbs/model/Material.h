#ifndef SBS_MODEL_MATERIAL_H_
#define SBS_MODEL_MATERIAL_H_

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

class Material {
public:
	Material(void);
	~Material(void);
	int LoadTextureFromBmp(const char *path);
	void Bind(void) const;
	void Unbind(void) const;
private:
	GLuint mDiffuseTexture;
};

#endif
