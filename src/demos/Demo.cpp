#include "common.h"
#include "GLFWApplication.h"
#include "engine/MeshGenerator.h"
#include "renderer/Renderer.h"
#include "renderer/Camera.h"
#include "engine/SoftBody.h"
#include "engine/CUDASoftBodySolver.h"

#include <iostream>
#include <glm/glm.hpp>

using namespace std;
using namespace glm;

#define SIZE(x) (sizeof(x)/sizeof(x[0]))

const int width = 800;
const int height = 600;

const vec3 particles[] = {
    vec3(1.000000, -1.000000, -1.000000),
    vec3(1.000000, -1.000000, 1.000000),
    vec3(-1.000000, -1.000000, 1.000000),
    vec3(-1.000000, -1.000000, -1.000000),
    vec3(1.000000, 1.000000, -0.999999),
    vec3(0.999999, 1.000000, 1.000001),
    vec3(-1.000000, 1.000000, 1.000000),
    vec3(-1.000000, 1.000000, -1.000000)
};

const uvec2 links[] = {
    uvec2(0,1),
    uvec2(1,2),
    uvec2(2,3),
    uvec2(3,0),
    uvec2(4,5),
    uvec2(5,6),
    uvec2(6,7),
    uvec2(7,4),
    uvec2(0,4),
    uvec2(1,5),
    uvec2(2,6),
    uvec2(3,7),

    uvec2(0,6),
    uvec2(1,7),
    uvec2(2,4),
    uvec2(3,5),
};

const uvec2 faces[] = {
    uvec2(1, 0), uvec2(6, 0), uvec2(2, 0),
    uvec2(1, 0), uvec2(4, 0), uvec2(8, 0),
    uvec2(1, 0), uvec2(8, 0), uvec2(5, 0),
    uvec2(1, 0), uvec2(5, 0), uvec2(6, 0),
    uvec2(2, 0), uvec2(6, 0), uvec2(3, 0),
    uvec2(3, 0), uvec2(8, 0), uvec2(4, 0),
    uvec2(3, 0), uvec2(6, 0), uvec2(7, 0),
    uvec2(3, 0), uvec2(7, 0), uvec2(8, 0),
    uvec2(6, 0), uvec2(8, 0), uvec2(7, 0),
    uvec2(5, 0), uvec2(8, 0), uvec2(6, 0),
    uvec2(1, 0), uvec2(2, 0), uvec2(3, 0),
    uvec2(1, 0), uvec2(3, 0), uvec2(4, 0),
};

class Demo : public GLFWApplication
{
public:
    Demo(int argc, char **argv);
    ~Demo(void);
    void OnKeyboard(int k);
    void OnMouseClick(GLFWApplication::ButtonType type, GLFWApplication::ButtonState state, int x, int y);
    void OnMouseMove(int x, int y);
    void OnRender(void);
    void OnUpdate(double dt);
private:
    SoftBodyRenderer    renderer;
    softbodyList_t     mSoftBodies;
    int mMouseLastX;
    int mMouseLastY;
    bool mMousePressed;
    Camera mCamera;
    CUDASoftBodySolver mSolver;
	int	mEnginUpdateTime;
};

Demo::Demo(int argc, char **argv) :
    GLFWApplication("DemoApp", width, height),
    mCamera(vec3(0,0,8), vec3(0,0,0), vec3(0,1,0)),
	mSolver()
{
	Cube cub(vec3(0,0,0), vec3(1, 1, -1));
	MeshData *md = MeshGenerator::generateFromCube(cub, 3, 3, 3);

	WRN("count: %d", md->vertexes.size());
	FOREACH_R(it, md->vertexes)
		WRN("%f, %f, %f", it->position[0], it->position[1], it->position[2]);

	WRN("count: %d", md->faces.size());
	FOREACH_R(it, md->faces)
		WRN("[%u, %u, %u]", (*it)[0], (*it)[1], (*it)[2]);

    vec3Array_t particlesA(particles, particles + SIZE(particles));
    index2Array_t linksA(links, links + SIZE(links));

    facesArray_t facesA;
#define FACE_ADD(x, arr, s, e) x.push_back(vector<uvec2>(&arr[s], &arr[s] + e ))

    FACE_ADD(facesA, faces, 0, 3);
    FACE_ADD(facesA, faces, 3, 3);
    FACE_ADD(facesA, faces, 6, 3);
    FACE_ADD(facesA, faces, 9, 3);
    FACE_ADD(facesA, faces, 12, 3);
    FACE_ADD(facesA, faces, 15, 3);
    FACE_ADD(facesA, faces, 18, 3);
    FACE_ADD(facesA, faces, 21, 3);
    FACE_ADD(facesA, faces, 24, 3);
    FACE_ADD(facesA, faces, 27, 3);
    FACE_ADD(facesA, faces, 30, 3);
    FACE_ADD(facesA, faces, 33, 3);
    
    SoftBody *b = new SoftBody(1.0f, 0.1f, 1.0f, &particlesA, &linksA, NULL, NULL, &facesA, VertexBuffer::OPENGL_BUFFER);
    mSoftBodies.push_back(b);

    renderer.initialize(width, height);
    renderer.setRenderMethod(SB_RENDER_PARTICLES);

	mSolver.addSoftBodies(mSoftBodies);
    mSolver.initialize();
}

Demo::~Demo(void)
{
    FOREACH(b, &mSoftBodies)
        delete *b;
}

#define ENGINE_TIME_STEP 15
#define DIFF_MAX 200

void Demo::OnUpdate(double dt)
{
	mSolver.projectSystem(dt);
}

void Demo::OnRender(void)
{
	mSolver.updateVertexBuffers();

	renderer.clearScreen();
	FOREACH(b, &mSoftBodies)
		renderer.renderBody(*b, mCamera.getCameraMatrix());
}

void Demo::OnKeyboard(int key)
{
    float angle = 2.0f;
    float delta = 0.1f;

    if (key == 'i')
		mSolver.projectSystem((float)ENGINE_TIME_STEP / 1000.0f);
    if (key == 'w')
        mCamera.moveUp(angle);
    if (key == 's')
        mCamera.moveDown(angle);
    if (key == 'e')
        mCamera.moveIn(delta);
    if (key == 'q')
        mCamera.moveOut(delta);
    if (key == 'm')
    {
        switch (renderer.getRenderMethod()) {
            case SB_RENDER_PARTICLES:
                renderer.setRenderMethod(SB_RENDER_FACES);
                break;
            case SB_RENDER_FACES:
                renderer.setRenderMethod(SB_RENDER_PARTICLES);
                break;
        }
    }
    if (key == 'd')
        mCamera.moveRight(angle);
    else if (key == 'a')
        mCamera.moveLeft(angle);
}

void Demo::OnMouseClick(ButtonType type, ButtonState state, int x, int y)
{
    if (type != LEFT_BUTTON)
        return;

    if (state == PRESSED) {
        mMouseLastX = x;
        mMouseLastY = y;
        mMousePressed = true;
    }
    else if (state == RELEASED)
        mMousePressed = false;
}

void Demo::OnMouseMove(int x, int y)
{
    if (!mMousePressed)
        return;

    static float angle = 0.2f;
    int dx = x - mMouseLastX;
    int dy = y - mMouseLastY;

    vec3 np;
    if (dx > 0)
        mCamera.moveRight(angle * dx);
    else
        mCamera.moveLeft(-angle * dx);
    if (dy > 0)
        mCamera.moveUp(angle * dy);
    else
        mCamera.moveDown(-angle * dy);

    mMouseLastX = x;
    mMouseLastY = y;
}

int main(int argc, char **argv)
{
    Demo demo(argc, argv);
    demo.MainLoop(0.5);
}
