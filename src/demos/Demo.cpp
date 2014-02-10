#include "GLUTApplication.h"
#include "Renderer.h"
#include "SoftBody.h"
#include "CUDASoftBodySolver.h"
#include <iostream>
#include <glm/glm.hpp>
#include "Camera.h"
#include "common.h"

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
    uvec2(0,5),
    uvec2(5,1),
    uvec2(1,0),
    uvec2(0,3),
    uvec2(3,7),
    uvec2(7,0),
    uvec2(2,6),
    uvec2(3,7),
    uvec2(4,5),
    uvec2(5,6),
    uvec2(6,7),
    uvec2(7,4),
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

class Demo : public GLUTApplication
{
public:
    Demo(int argc, char **argv);
    ~Demo(void);
    void onKeyboard(unsigned char k, int x, int y);
    void onMouseClick(ButtonType type, ButtonState state, int x, int y);
    void onMouseMove(int x, int y);
    void onMenuSelected(string &s);
    void onDisplay(void);
private:
    SoftBodyRenderer    renderer;
    softbodyArray_t     mSoftBodies;
    int mMouseLastX;
    int mMouseLastY;
    bool mMousePressed;
    Camera mCamera;
    CUDASoftBodySolver mSolver;
};

Demo::Demo(int argc, char **argv) :
    GLUTApplication(argc, argv, "DemoApp", width, height),
    mCamera(vec3(0,0,8), vec3(0,0,0), vec3(0,1,0))
{
    initialize();

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
    
    SoftBody *b = new SoftBody(1.0f, 1.0f, 1.0f, &particlesA, &linksA, NULL, NULL, &facesA, VertexBuffer::OPENGL_BUFFER);
    mSoftBodies.push_back(b);

    renderer.initialize(width, height);
    renderer.setRenderMethod(SB_RENDER_PARTICLES);

    mSolver.initialize(&mSoftBodies);

    int id1 = addMenu("Main");
    addMenuEntry(id1, "Hello1");
    addMenuEntry(id1, "Hello2");

    int id2 = addMenu("SubMenu", id1);
    addMenuEntry(id2, "Hello3");
    addMenuEntry(id2, "Hello4");

    attachMenu(id1, RIGHT_BUTTON);
}

Demo::~Demo(void)
{
    FOREACH(b, &mSoftBodies)
        delete *b;
}

void Demo::onDisplay(void)
{
	mSolver.projectSystem(0.03f);
	mSolver.updateVertexBuffers();
    renderer.clearScreen();

    FOREACH(b, &mSoftBodies)
        renderer.renderBody(*b, mCamera.getCameraMatrix());
}

void Demo::onKeyboard(unsigned char key, int x, int y)
{
    float angle = 2.0f;
    float delta = 0.1f;

	if (key == 'u') {
		mSolver.projectSystem(0.01f);
		mSolver.updateVertexBuffers();
	}
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

void Demo::onMouseClick(ButtonType type, ButtonState state, int x, int y)
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

void Demo::onMouseMove(int x, int y)
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

void Demo::onMenuSelected(string &s)
{
    cout << s << endl;
}

int main(int argc, char **argv)
{
    Demo demo(argc, argv);
    demo.run();
}
