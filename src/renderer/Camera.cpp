#include "Camera.h"

#include <math.h>
#include <glm/ext.hpp>

using namespace glm;

Camera::Camera(vec3 position, vec3 lookat, vec3 up)
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
	_position = rotate(_position, angle, _up);
}

void Camera::moveRight(float_t angle)
{
	_position = rotate(_position, -angle, _up);
}

void Camera::moveUp(float_t angle)
{
	vec3 rotVec = cross((_position - _lookAtPoint), _up);
	_position = rotate(_position, angle, rotVec);
}

void Camera::moveDown(float_t angle)
{
	vec3 rotVec = cross((_position - _lookAtPoint), _up);
	_position = rotate(_position, -angle, rotVec);
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
	_matrix = lookAt(_position, _lookAtPoint, _up);
	return _matrix;
}

