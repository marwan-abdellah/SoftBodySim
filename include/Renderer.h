#ifndef __SOFTBODY_RENDERER_H
#define __SOFTBODY_RENDERER_H

class SoftBodyRenderer {
	void setUp();
	void clearScreen(void);
	void draw(SoftBody &body);
	void draw(ColliionBody &body);
	void tearDown();
};

#endif
