#ifndef SB_ENGINE_BOX_H_
#define SB_ENGINE_BOX_H_

#include <glm/fwd.hpp>

/*
 * @brief Creates an Box aligned with x, y, z axis
 */
struct Box {
	/**
	 * @brief Constructor
	 *
	 * @param[in] bl Bottom left front corner of box in RHS coords
	 * @param[in] ur Upper right back corner of box in RHS coords.
	 *
	 * @remark RHS (Right Hand Screen coordinates) assumets Z-axis
	 * pointing out of screen.
	 */
	Box(glm::vec3 bl, glm::vec3 ur) :
		bottomLeftFront(bl), upperRightBack(ur)
	{}

	/**
	 * Copy constructor
	 */
	Box(const Box &p)
		: bottomLeftFront(p.bottomLeftFront), upperRightBack(p.upperRightBack) {}
	~Box(void) {}

	glm::vec3 bottomLeftFront;
	glm::vec3 upperRightBack;
};

#endif
