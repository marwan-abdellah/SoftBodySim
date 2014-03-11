#ifndef __CAMERA_H
#define __CAMERA_H

#include <cmath>
#include <glm/glm.hpp>

class Camera {
	public:
		Camera(glm::vec3 position, glm::vec3 point, glm::vec3 lookAt);
		void setLookAtPoint(glm::vec3 &point);
		void moveLeft(float rad);
		void moveRight(float rad);
		void moveUp(float rad);
		void moveDown(float rad);
		void moveIn(float in);
		void moveOut(float out);
		const glm::mat4 *getCameraMatrix(void);
		~Camera() {}
	private:
		glm::vec3	_position;
		glm::vec3	_up;
		glm::vec3	_lookAtPoint;
		glm::mat4 	_matrix;
};

#endif

