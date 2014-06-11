#include "Math.h"
#include "common.h"

#include <glm/ext.hpp>

using namespace glm;

static void inline rotate(mat3 &mat, double &c, double &s, int i0, int j0, int i1, int j1)
{
	double a = c * mat[i0][j0] - s * mat[i1][j1];
	double b = s * mat[i0][j0] + c * mat[i1][j1];
	mat[i0][j0] = a;
	mat[i1][j1] = b;
}

vec3 eigenvalues_jacobi(mat3 &mat, int max_iter, mat3 &E)
{
	vec3 ret;
	bool changed = true;

	// initial eigenvalues
	ret[0] = mat[0][0];
	ret[1] = mat[1][1];
	ret[2] = mat[2][2];

	E = mat3();
	mat[0][0] = 1.0;
	mat[1][1] = 1.0;
	mat[2][2] = 1.0;

	REP(z, max_iter) {
		changed = false;
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
					changed = true;
					ret[i] = mii1;
					ret[j] = mjj1;
					mat[i][j] = 0.0;
					for(unsigned int k = 0; k < i; k++) rotate(mat, c, s, k, i, k, j);
					for(int k = i + 1; k < j; k++) rotate(mat, c, s, i, k, k, j);
					for(int k = j + 1; k < 3; k++) rotate(mat, c, s, i, k, j, k);
					for(int k = 0; k < 3; k++) rotate(E, c, s, k, i, k, j);
				}
			}
		}
		if (!changed) break;
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
void polar_decomposition(const mat3 &A, mat3 &R, mat3 &S)
{
	mat3 E;

	mat3 B = transpose(A) * A;

	// B is symmetrix matrix so it is diagonizable
	vec3 eig = eigenvalues_jacobi(B, 10, E);

	// add delta value to eigenvalues to overcome det(A) == 0 problem
	eig += vec3(0.001f, 0.001f, 0.001f);

	S = glm::diagonal3x3(eig);

	// calculate squere root of diagonal matrix
	S[0][0] = sqrt(S[0][0]);
	S[1][1] = sqrt(S[1][1]);
	S[2][2] = sqrt(S[2][2]);

	// calcuate squre root of B matrix
	B = inverse(E) * S * E;

	// calculate rotation matrix
	R = A * inverse(B);

	float_t det = glm::determinant(R);
	if (det != det) {
		ERR("det(A): %f", glm::determinant(A));
		ERR("det(R): %f", det);
		ERR("%f %f %f", A[0][0], A[0][1], A[0][2]);
		ERR("%f %f %f", A[1][0], A[1][1], A[1][2]);
		ERR("%f %f %f", A[2][0], A[2][1], A[2][2]);
	}
}
