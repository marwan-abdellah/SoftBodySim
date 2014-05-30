#include "Material.h"
#include "common.h"
#include <fstream>

using namespace std;

Material::Material() :
	mDiffuseTexture(0)
{
	glGenTextures(1, &mDiffuseTexture);
}

Material::~Material()
{
	glDeleteTextures(1, &mDiffuseTexture);
}

void Material::Bind(void)
{
	if (!mDiffuseTexture) return;
	glBindTexture(GL_TEXTURE_2D, mDiffuseTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void Material::Unbind(void)
{
	if (!mDiffuseTexture) return;
	glBindTexture(GL_TEXTURE_2D, 0);
}

int Material::LoadTextureFromBmp(const char *path)
{
    // source:
	//http://www.opengl-tutorial.org/beginners-tutorials/tutorial-5-a-textured-cube/
	char header[54];
	unsigned int dataPos;
	unsigned int width, height;
	unsigned int imageSize;
	char *data;

	fstream file;
	file.open(path);
	if (!file.is_open()) {
		ERR("Unable to open file: %s", path);
		return -1;
	}
	DBG("Reading BMP header...");
	file.read(header, (int)sizeof(header)/sizeof(header[0]));
	
	if (!file) {
		ERR("Unable to read header of %s", path);
		file.close();
		return -1;
	}
	if (header[0] != 'B' || header[1] != 'M') {
		ERR("Not a BMP file!");
		file.close();
		return -1;
	}
	dataPos = *(int*)&(header[0x0A]);
	imageSize = *(int*)&(header[0x22]);
	width = *(int*)&(header[0x12]);
	height = *(int*)&(header[0x16]);

	if (imageSize==0) imageSize=width*height*3;
	if (dataPos==0) dataPos=54;

	data = new char[imageSize];
	file.read(data, imageSize);
	if (!file) {
		ERR("Unable to read data of %s", path);
		delete data;
		file.close();
		return -1;
	}
	file.close();

	glBindTexture(GL_TEXTURE_2D, mDiffuseTexture);

    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, data);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	delete data;
	glBindTexture(GL_TEXTURE_2D, 0);

	DBG("Texture loaded: w: %d h:%d size:%d", width, height, imageSize);
	return 0;
}
