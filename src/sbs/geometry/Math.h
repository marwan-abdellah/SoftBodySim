#ifndef SBS_ENGINE_MATH_H_
#define SBS_ENGINE_MATH_H_

#include <glm/fwd.hpp>

/**
 * Diagonize matrix using Jacobi rotations. 
 * @remark Method does not check if matrix is diagonizable.
 * Passing non diagonizable matrix and infinite max_iter (= -1)
 * May result in infinite loop.
 */
glm::vec3 eigenvalues_jacobi(glm::mat3 &mat, int max_iter, glm::mat3 &m);

void polar_decomposition(const glm::mat3 &A, glm::mat3 &R, glm::mat3 &S);


#endif
