#include "common.h"

#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "Camera.h"

Camera::Camera(glm::vec3 position, glm::vec3 lookat, glm::vec3 up)
{
	_position = position;
	_lookAtPoint = lookat;
	_up = up;
}

void Camera::setLookAtPoint(glm::vec3 &point)
{
	_lookAtPoint = point;
}

void Camera::moveLeft(float_t angle)
{
	_position = glm::rotate(_position, angle, _up);
}

void Camera::moveRight(float_t angle)
{
	_position = glm::rotate(_position, -angle, _up);
}

void Camera::moveUp(float_t angle)
{
	glm::vec3 rotVec = glm::cross((_position - _lookAtPoint), _up);
	_position = glm::rotate(_position, angle, rotVec);
}

void Camera::moveDown(float_t angle)
{
	glm::vec3 rotVec = glm::cross((_position - _lookAtPoint), _up);
	_position = glm::rotate(_position, -angle, rotVec);
}

void Camera::moveIn(float_t in)
{
	//FIXME normalize?
	_position -= (_position - _lookAtPoint) * in;
}

void Camera::moveOut(float_t out)
{
	//FIXME normalize?
	_position += (_position - _lookAtPoint) * out;
}

const glm::mat4 &Camera::getCameraMatrix(void)
{
	_matrix = glm::lookAt(_position, _lookAtPoint, _up);
	return _matrix;
}
