#include "GLUTApplication.h"
#include <GL/freeglut.h>
#include <iostream>

GLUTApplication *GLUTApplication::active_instance = NULL;

using namespace std;

int GLUTApplication::getElapsedTime(void)
{
    return glutGet(GLUT_ELAPSED_TIME);
}

void GLUTApplication::syncScreen(void)
{
    glutSwapBuffers();
    glutPostRedisplay();
}

void GLUTApplication::onDisplayHelper(void)
{
    if (active_instance)
        active_instance->onDisplay();
}

void GLUTApplication::onIdleHelper()
{
    if (active_instance)
        active_instance->onIdle();
}

void GLUTApplication::onKeyboardHelper(unsigned char k, int x, int y)
{
    if (active_instance)
        active_instance->onKeyboard(k, x, y);
}

void GLUTApplication::onKeyboardUpHelper(unsigned char k, int x, int y)
{
    if (active_instance)
        active_instance->onKeyboardUp(k, x, y);
}

void GLUTApplication::onMouseClickHelper(int key, int state, int x, int y)
{
    ButtonType type;
    ButtonState st;
    switch (key) {
        case GLUT_LEFT_BUTTON:
            type = LEFT_BUTTON;
            break;
        case GLUT_MIDDLE_BUTTON:
            type = MIDDLE_BUTTON;
            break;
        case GLUT_RIGHT_BUTTON:
            type = RIGHT_BUTTON;
            break;
    }

    switch (state) {
        case GLUT_DOWN:
            st = PRESSED;
            break;
        case GLUT_UP:
            st = RELEASED;
            break;
    }

    if (active_instance)
        active_instance->onMouseClick(type, st, x, y);
}

void GLUTApplication::onMouseMoveHelper(int x, int y)
{
    if (active_instance)
        active_instance->onMouseMove(x, y);
}

void GLUTApplication::onMenuSelectedHelper(int id)
{
    if (active_instance)
        active_instance->onMenuSelected(active_instance->m_menuKeys[id]);
}

GLUTApplication::GLUTApplication(int argc, char **argv, const char *title, int width, int height) :
    m_argc(argc),
    m_argv(argv),
    m_title(title),
    m_width(width),
    m_height(height)
{
}

GLUTApplication::~GLUTApplication()
{
    shutdown();
}

void GLUTApplication::attachMenu(int menuId, ButtonType button)
{
    glutSetMenu(menuId);
    int bid;
    switch (button) {
        case LEFT_BUTTON:
            bid = GLUT_LEFT_BUTTON;
            break;
        case RIGHT_BUTTON:
            bid = GLUT_RIGHT_BUTTON;
            break;
        case MIDDLE_BUTTON:
            bid = GLUT_MIDDLE_BUTTON;
    }
    glutAttachMenu(bid);
}

void GLUTApplication::addMenuEntry(int menuId, string entry)
{
    glutSetMenu(menuId);
    glutAddMenuEntry(entry.c_str(), m_menuKeys.size());
    m_menuKeys.push_back(entry);
}

int GLUTApplication::addMenu(string entry, int parentMenuId)
{
    int id;
    id = glutCreateMenu(GLUTApplication::onMenuSelectedHelper);

    if (parentMenuId != -1) {
        glutSetMenu(parentMenuId);
        glutAddSubMenu(entry.c_str(), id);
    }

    return id;
}

void GLUTApplication::initialize(void)
{
    if (!active_instance) {
        glutInit(&m_argc, m_argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
        glutInitWindowSize(m_width, m_height);
        glutCreateWindow(m_title.c_str());

        glutKeyboardFunc(GLUTApplication::onKeyboardHelper);
        glutKeyboardUpFunc(GLUTApplication::onKeyboardUpHelper);
        glutMouseFunc(GLUTApplication::onMouseClickHelper);
        glutMotionFunc(GLUTApplication::onMouseMoveHelper);
        glutIdleFunc(GLUTApplication::onIdleHelper);
        glutDisplayFunc(GLUTApplication::onDisplayHelper);

        active_instance = this;
    }
}

void GLUTApplication::shutdown(void)
{
    if (active_instance != this)
        return;
    glutLeaveMainLoop();
    active_instance = NULL;
}

void GLUTApplication::run(void)
{
    if (active_instance != this)
        return;
    glutMainLoop();
}
