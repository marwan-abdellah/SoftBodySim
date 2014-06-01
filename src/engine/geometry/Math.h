#ifndef MATH_H_
#define MATH_H_

#include <glm/glm.hpp>

/**
 * Diagonize matrix using Jacobi rotations. 
 * @remark Method does not check if matrix is diagonizable.
 * Passing non diagonizable matrix and infinite max_iter (= -1)
 * May result in infinite loop.
 */
glm::vec3 eigenvalues_jacobi(glm::mat3 &mat, int max_iter);


#endif
