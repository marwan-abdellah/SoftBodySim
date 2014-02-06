#include "GLUTApplication.h"
#include "Renderer.h"
#include <iostream>

using namespace std;

class Demo : public GLUTApplication
{
public:
    Demo(int argc, char **argv);
    ~Demo(void) {}
    void onKeyboard(unsigned char k, int x, int y);
    void onMouseClick(int key, int state, int x, int y);
    void onMouseMove(int x, int y);
    void onMenuSelected(string &s);
};

Demo::Demo(int argc, char **argv) :
    GLUTApplication(argc, argv, "DemoApp", 400, 400)
{
    initialize();

    int id1 = addMenu("Main");
    addMenuEntry(id1, "Hello1");
    addMenuEntry(id1, "Hello2");

    int id2 = addMenu("SubMenu", id1);
    addMenuEntry(id2, "Hello3");
    addMenuEntry(id2, "Hello4");

    attachMenu(id1, RIGHT_BUTTON);
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
