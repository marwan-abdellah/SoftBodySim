#ifndef SB_BODY_H
#define SB_BODY_H

#include "geometry/Sphere.h"


/**
 * @class Abstract class 
 */
class Body {
public:
	Body(void) : mVertexes(NULL), mEdges(NULL), mFaces(NULL) {}
	virtual const Sphere *getBoundingVolume(void) = 0;

    const VertexBuffer *GetVertexes(void) { return mVertexes; }
	const ElementBuffer *getEdges(void) { return mEdges; }
	const ElementBuffer *getFaces(void) { return mFaces; }

protected:
	VertexBuffer				*mVertexes;
    ElementBuffer               *mEdges;
    ElementBuffer               *mFaces;
};

#endif
