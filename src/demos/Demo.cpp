#include "GLUTApplication.h"
#include "Renderer.h"
#include "SoftBody.h"
#include <iostream>
#include <glm/glm.hpp>
#include "Camera.h"
#include "common.h"

using namespace std;
using namespace glm;

#define SIZE(x) (sizeof(x)/sizeof(x[0]))


static Camera camera(vec3(0,0,8), vec3(0,0,0), vec3(0,1,0));
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
    void onMouseClick(int key, int state, int x, int y);
    void onMouseMove(int x, int y);
    void onMenuSelected(string &s);
    void onDisplay(void);
private:
    SoftBodyRenderer    renderer;
    SoftBody        *b;
};

Demo::Demo(int argc, char **argv) :
    GLUTApplication(argc, argv, "DemoApp", width, height)
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
    
    b = new SoftBody(1,1,1, &particlesA, &linksA, NULL, NULL, &facesA, VertexBuffer::OPENGL_BUFFER);

    renderer.initialize(width, height);
    renderer.setRenderMethod(SB_RENDER_PARTICLES);

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
    if (b) delete b;
}

void Demo::onDisplay(void)
{
    static int d;
    renderer.clearScreen();
    renderer.renderBody(b, camera.getCameraMatrix());

    if (!d) {
        cout << "onDisplay" << endl;
        d = 1;
    }
}

void Demo::onKeyboard(unsigned char k, int x, int y)
{
    cout << "OnKeyboard : " << k << endl;
    if (k == 'q')
        shutdown();
}

void Demo::onMouseClick(int key, int state, int x, int y)
{
    cout << "OnClick" << endl;
}

void Demo::onMouseMove(int x, int y)
{
    cout << "OnMouseMove" << endl;
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
