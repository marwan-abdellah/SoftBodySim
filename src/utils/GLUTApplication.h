#include <GL/glut.h>

class GLUTApplication
{
public:
	GLUTApplication(int argc, char *arg, const char *title, int width, int height);
	~GLUTApplication(void);

	virtual void initialize(void) = 0;
	virtual void shutdown(void) = 0;

	virtual void onIdle(void) = 0;
	virtual void onDisplay(void) = 0;
	virtual void onKeyboardDown(char k) = 0;
	virtual void onKeyboardUp(char k) = 0;
	virtual void onMouseClick(int key, int state, int x, int y) = 0;

	void run(void);
}
