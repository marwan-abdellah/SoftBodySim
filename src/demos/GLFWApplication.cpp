#include <GLFWApplication.h>
#include <cstdlib>
#include <iostream>

static GLFWApplication *app;

#define MAX_FRAME_TIME 0.5

static void errorCb(int error, const char *descr)
{
	std::cerr << descr << std::endl;
}

void keyCb(GLFWwindow *m_window, int key, int scancode, int action, int mds)
{
	if (!app) return;
	app->OnKeyboard(key, action);
}

void mouseMoveCb(GLFWwindow *m_window, double x, double y)
{
	if (!app) return;
	app->OnMouseMove(x, y);
}

void mouseCb(GLFWwindow *m_window, int button, int action,
		int mods)
{
	double x, y;
	glfwGetCursorPos(app->m_window, &x, &y);
	app->OnMouseClick(button, action, x, y);
}

void scrollCb(GLFWwindow *m_window, double x, double y)
{
	app->OnScroll(x, y);
}

GLFWApplication::GLFWApplication(const char *title, unsigned int width, unsigned int height) :
	m_lastFrameTime(0),
	m_width(width),
	m_height(height),
	m_title(title)
{
	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwSetErrorCallback(errorCb);

	m_window = glfwCreateWindow(m_width, m_height, m_title.c_str(), NULL, NULL);
	if (!m_window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(m_window);
	glfwSetKeyCallback(m_window, keyCb);
	glfwSetCursorPosCallback(m_window, mouseMoveCb);
	glfwSetMouseButtonCallback(m_window, mouseCb);
	glfwSetScrollCallback(m_window, scrollCb);
}

GLFWApplication::~GLFWApplication()
{
	glfwTerminate();
}

void GLFWApplication::MainLoop(double delta)
{
	double accumulator = 0.0;
	m_lastFrameTime = glfwGetTime();

	app = this;

	while (!glfwWindowShouldClose(m_window)) {
		double currentTime = glfwGetTime();
		double frameTime = currentTime - m_lastFrameTime;

		if (frameTime > MAX_FRAME_TIME)
			frameTime = MAX_FRAME_TIME;

		accumulator += frameTime;
		m_lastFrameTime = currentTime;

		while (accumulator >= delta) {
			OnUpdate(delta);
			accumulator -= delta;
		}

		OnRender();
		glfwSwapBuffers(m_window);
		glfwPollEvents();
	}

	app = NULL;
	glfwDestroyWindow(m_window);
}

void GLFWApplication::QuitMain(void)
{
	glfwSetWindowShouldClose(m_window, GL_TRUE);
}
