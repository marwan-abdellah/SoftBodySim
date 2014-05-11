#include "common.h"
#include "GLFWApplication.h"
#include "engine/MeshData.h"
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
	MeshData md = MeshData::CreateCube(vec3(-1,-1, 1), vec3(1, 1, -1), 3, 3, 3);
    
    SoftBody *b = new SoftBody(1.0f, 0.1f, 1.0f, md);
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
    demo.MainLoop(0.02);
}
