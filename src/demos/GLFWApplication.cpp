#include <GLFWApplication.h>
#include <cstdlib>
#include <iostream>

static GLFWApplication *app;

static void errorCb(int error, const char *descr)
{
	std::cout << descr << std::endl;
}

void keyCb(GLFWwindow *m_window, int key, int scancode, int action, int mds)
{
	if (!app) return;
	app->OnKeyboard(key);
}

void mouseMoveCb(GLFWwindow *m_window, double x, double y)
{
	if (!app) return;
	app->OnMouseMove(x, y);
}

void mouseCb(GLFWwindow *m_window, int button, int action,
		int mods)
{
	GLFWApplication::ButtonType type;
	GLFWApplication::ButtonState state;
	double x, y;
	if (!app) return;
	switch (button) {
		case GLFW_MOUSE_BUTTON_1:
			type = GLFWApplication::LEFT_BUTTON;
			break;
		case GLFW_MOUSE_BUTTON_2:
			type = GLFWApplication::RIGHT_BUTTON;
			break;
		default:
			return;
	}
	switch (action) {
		case GLFW_PRESS:
			state = GLFWApplication::PRESSED;
			break;
		case GLFW_RELEASE:
			state = GLFWApplication::RELEASED;
			break;
		default:
			return;
	}
	glfwGetCursorPos(app->m_window, &x, &y);
	app->OnMouseClick(type, state, x, y);
}

GLFWApplication::GLFWApplication(const char *title, unsigned int width, unsigned int height) :
	m_currentTime(0),
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
}

GLFWApplication::~GLFWApplication()
{
	glfwTerminate();
}

void GLFWApplication::MainLoop(double delta)
{
	double accumulator = 0.0;

	app = this;

	while (!glfwWindowShouldClose(m_window)) {
		double newTime = glfwGetTime();
		double frameTime = newTime - m_currentTime;

		if (frameTime > 0.5)
			frameTime = 0.5;

		m_currentTime = newTime;

		while (accumulator >= delta) {
			OnUpdate( delta );
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
