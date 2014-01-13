#include <string>
#include <vector>

#ifndef _GLUT_APP__H
#define _GLUT_APP__H

class GLUTApplication
{
public:
	virtual ~GLUTApplication();
	GLUTApplication(int argc, char **argv, const char *title, int width, int height);
	void initialize(void);
	void shutdown(void);
	void run(void);

	enum Button {
		RIGHT_BUTTON,
		MIDDLE_BUTTON,
		LEFT_BUTTON
	};

	void attachMenu(int menuId, Button button);
	void addMenuEntry(int menuId, std::string entry);
	int addMenu(std::string entry, int parentMenuId=-1);

	virtual void onDisplay(void) {}
	virtual void onKeyboardUp(unsigned char k, int x, int y) {}
	virtual void onKeyboard(unsigned char k, int x, int y) {}
	virtual void onMouseClick(int key, int state, int x, int y) {}
	virtual void onMouseMove(int x, int y) {}
	virtual void onMenuSelected(std::string &) {}
protected:
	int m_argc;
	char **m_argv;
private:
	std::string m_title;
	int m_width, m_height;
	std::vector<std::string> m_menuKeys;
	GLUTApplication(const GLUTApplication&) {}
	static GLUTApplication *active_instance;
	static void onKeyboardHelper(unsigned char, int, int);
	static void onKeyboardUpHelper(unsigned char, int x, int y);
	static void onMouseClickHelper(int, int, int, int);
	static void onMouseMoveHelper(int, int);
	static void onIdleHelper(void);
	static void onDisplayHelper(void);
	static void onMenuSelectedHelper(int id);
};

#endif
