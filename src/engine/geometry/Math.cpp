#include "Math.h"
#include "common.h"

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
