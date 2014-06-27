#ifndef SBS_ENGINE_MATH_H_
#define SBS_ENGINE_MATH_H_

#include "common.h"
#include <glm/glm.hpp>

/**
 * This is defined in glm/ext.hpp but for whatever reason ext.hpp can't be
 * included by CUDA code, so I decided to redefine it.
 */
GLM_FUNC_QUALIFIER glm::mat3 diagonal3x3(glm::vec3 &v)
{
	glm::mat3 ret(0.0f);
	ret[0][0] = v[0];
	ret[1][1] = v[1];
	ret[2][2] = v[2];
	return ret;
}

GLM_FUNC_QUALIFIER void eigenvalues_rotate(glm::mat3 &mat, double &c, double &s, int i0, int j0, int i1, int j1)
{
	double a = c * mat[i0][j0] - s * mat[i1][j1];
	double b = s * mat[i0][j0] + c * mat[i1][j1];
	mat[i0][j0] = a;
	mat[i1][j1] = b;
}

/**
 * Diagonize matrix using Jacobi rotations. 
 * @remark Method does not check if matrix is diagonizable.
 * Passing non diagonizable matrix and infinite max_iter (= -1)
 * May result in infinite loop.
 */
GLM_FUNC_QUALIFIER glm::vec3 eigenvalues_jacobi(glm::mat3 &mat, int max_iter, glm::mat3 &E)
{
	glm::vec3 ret;

	// initial eigenvalues
	ret[0] = mat[0][0];
	ret[1] = mat[1][1];
	ret[2] = mat[2][2];

	E = glm::mat3();
	mat[0][0] = 1.0;
	mat[1][1] = 1.0;
	mat[2][2] = 1.0;

	REP(z, max_iter) {
		REP(i, 3) {
			for(int j = i + 1; j < 3; j++) {
				double mii = ret[i];
				double mjj = ret[j];
				double mij = mat[i][j];
				double phi = 0.5 * atan2(2 * mij, mjj - mii);
				double c = cos(phi);
				double s = sin(phi);
				double mii1 = c * c * mii - 2 * s * c * mij + s * s * mjj;
				double mjj1 = s * s * mii + 2 * s * c * mij + c * c * mjj;
				if (abs(mii - mii1) < 0.00001 || (mjj - mjj1) < 0.00001) {
//					changed = true;
					ret[i] = mii1;
					ret[j] = mjj1;
					mat[i][j] = 0.0;
					for(unsigned int k = 0; k < i; k++) eigenvalues_rotate(mat, c, s, k, i, k, j);
					for(int k = i + 1; k < j; k++) eigenvalues_rotate(mat, c, s, i, k, k, j);
					for(int k = j + 1; k < 3; k++) eigenvalues_rotate(mat, c, s, i, k, j, k);
					for(int k = 0; k < 3; k++) eigenvalues_rotate(E, c, s, k, i, k, j);
				}
			}
		}
	}
	return ret;
}

/**
 * Function for getting polar decomposition for well-defined inversable
 * matrixes (det(A) > 0)
 *
 * Polar decomposition is given as:
 * A = R * S,
 * where R is rotation matrix and S is a scale matrix along orthogonal axis.
 *
 * by definition 
 * R = A * S^-1
 * S = sqrt(A' * A)
 */
GLM_FUNC_QUALIFIER void polar_decomposition(const glm::mat3 &A, glm::mat3 &R, glm::mat3 &S)
{
	glm::mat3 E;
	glm::mat3 B = glm::transpose(A) * A;

	// B is symmetrix matrix so it is diagonizable
	glm::vec3 eig = eigenvalues_jacobi(B, 5, E);

	// add delta value to eigenvalues to overcome det(A) == 0 problem
	eig += glm::vec3(0.01f, 0.01f, 0.01f);

	S = diagonal3x3(eig);

	// calculate squere root of diagonal matrix
	S[0][0] = glm::sqrt(S[0][0]);
	S[1][1] = glm::sqrt(S[1][1]);
	S[2][2] = glm::sqrt(S[2][2]);

	// calcuate squre root of B matrix
	B = glm::inverse(E) * S * E;

	// calculate rotation matrix
	R = A * glm::inverse(B);

#ifndef __CUDA_ARCH__
	float_t det = glm::determinant(R);
	if (det != det) {
		ERR("det(A): %f", glm::determinant(A));
		ERR("det(R): %f", det);
		ERR("%f %f %f", A[0][0], A[0][1], A[0][2]);
		ERR("%f %f %f", A[1][0], A[1][1], A[1][2]);
		ERR("%f %f %f", A[2][0], A[2][1], A[2][2]);
		SB_ASSERT(false);
	}
#endif
}

GLM_FUNC_QUALIFIER glm::float_t triangle_area(glm::vec3 &a, glm::vec3 &b, glm::vec3 &c)
{
	glm::vec3 ab = b - a;
	glm::vec3 ac = c - a;
	return 0.5f * glm::length(glm::cross(ab, ac));
}

GLM_FUNC_QUALIFIER glm::float_t calculateVolume(glm::vec3 *pos, glm::uvec3 *triangles, glm::vec3 *norms, glm::uint_t *accum, int n)
{
	double ret = 0.0f;
	glm::vec3 norm;

	for(int i = 0; i < n; i++) {
		glm::vec3 v0 = pos[triangles[i][0]];
		glm::vec3 v1 = pos[triangles[i][1]];
		glm::vec3 v2 = pos[triangles[i][2]];
		norm = glm::cross(v1 - v0, v2 - v0);
		if (norm != glm::vec3(0,0,0)) {
			norm = glm::normalize(norm);
		}
		float_t area = triangle_area(v0, v1, v2);
		ret += area * glm::dot(v0 + v1 + v2, norm);

		if (norms) {
			norm = area * norm;
			norms[triangles[i][0]] += norm;
			norms[triangles[i][1]] += norm;
			norms[triangles[i][2]] += norm;
			accum[triangles[i][0]]++;
			accum[triangles[i][1]]++;
			accum[triangles[i][2]]++;
		}
	}

	return (glm::float_t)ret / 3.0f;
}

GLM_FUNC_QUALIFIER glm::vec3 calculateMassCenter(glm::vec3 *pos, glm::float_t *mass, int n)
{
	double masssum = 0.0;
	glm::vec3 xmsum = glm::vec3(0,0,0);

	//calculate sum(xi * mi) amd sum(mi)
	REP(i, n) {
		xmsum += pos[i] * mass[i];
		masssum += mass[i];
	}

	return xmsum / (glm::float_t)masssum;
}

#endif
