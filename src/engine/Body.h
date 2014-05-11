#ifndef SB_BODY_H
#define SB_BODY_H

#include "geometry/Sphere.h"


/**
 * @class Abstract class 
 */
class Body {
public:
	virtual const Sphere *getBoundingVolume(void) = 0;
};

#endif
