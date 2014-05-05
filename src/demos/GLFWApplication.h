#ifndef GLFW_APPLICATION_H
#define GLFW_APPLICATION_H

#include <GLFW/glfw3.h>
#include <string>

class GLFWApplication
{
public:
    enum ButtonType {
        RIGHT_BUTTON,
        MIDDLE_BUTTON,
        LEFT_BUTTON
    };

    enum ButtonState {
        PRESSED,
        RELEASED
    };

	GLFWApplication(const char *title, unsigned int width, unsigned int height);
	virtual ~GLFWApplication();

	virtual void OnUpdate(double dt) = 0;
	virtual void OnRender(void) = 0;

	virtual void OnMouseMove(int x, int y) {}
    virtual void OnMouseClick(ButtonType btn, ButtonState state, int x, int y) {}
	virtual void OnKeyboard(int key) {}

	void MainLoop(double timeDelta);
	void QuitMain(void);
protected:
	double m_currentTime;
	int m_width, m_height;
	std::string m_title;
private:
	GLFWwindow *m_window;
friend void keyCb(GLFWwindow *m_window, int key, int scancode, int action, int mds);
friend void mouseMoveCb(GLFWwindow *m_window, double x, double y);
friend void mouseCb(GLFWwindow *m_window, int button, int action, int mods);
};

#endif
