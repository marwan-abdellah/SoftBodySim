#include "Math.h"
#include "common.h"

using namespace glm;

#define ROTATE(mat, k, l, i, j) \

static void inline rotate(mat3 &mat, float_t &c, float &s, int i0, int j0, int i1, int j1)
{
	float_t a = c * mat[i0][j0] - s * mat[i1][j1];
	float_t b = s * mat[i0][j0] + c * mat[i1][j1];
	mat[i0][j0] = a;
	mat[i1][j1] = b;
}

vec3 eigenvalues_jacobi(mat3 &mat, int max_iter)
{
	vec3 ret;
	bool changed = true;

	// initial eigenvalues
	ret[0] = mat[0][0];
	ret[1] = mat[1][1];
	ret[2] = mat[2][2];

	REP(z, max_iter) {
		changed = false;
		REP(i, 3) {
			for(int j = i + 1; j < 3; j++) {
				float_t mii = ret[i];
				float_t mjj = ret[j];
				float_t mij = mat[i][j];
				float_t phi = 0.5 * atan2(2 * mij, mjj - mii);
				float_t c = cos(phi);
				float_t s = sin(phi);
				float_t mii1 = c * c * mii - 2 * s * c * mij + s * s * mjj;
				float_t mjj1 = s * s * mii + 2 * s * c * mij + c * c * mjj;
				if (mii != mii1 || mjj != mjj1) {
					changed = true;
					ret[i] = mii1;
					ret[j] = mjj1;
					mat[i][j] = 0.0;
					for(int k = 0; k < i; k++) rotate(mat, c, s, k, i, k, j);
					for(int k = i + 1; k < j; k++) rotate(mat, c, s, i, k, k, j);
					for(int k = j + 1; k < 3; k++) rotate(mat, c, s, i, k, j, k);
				}
			}
		}
		if (!changed) break;
	}
	return ret;
}

#if 0
vec3 eigenvalues_jacobi2(mat3 &mat, int max_iter)
{
	vec3 ret;
	int maxs[3];

	// find biggest off-diagonal elements
	maxs[0] = mat[0][1] > mat[0][2] ? 1 : 2;
	maxs[1] = mat[1][0] > mat[1][2] ? 0 : 2;
	maxs[2] = mat[2][0] > mat[2][1] ? 0 : 1;
	//maxs[1] = 1;
	//maxs[2] = 2;


	REP(i, max_iter) {
		// find biggest element out-off three row maxes
		int m = 0;
		m = mat[m][maxs[m]] > mat[1][maxs[1]] ? m : 1;
		m = mat[m][maxs[m]] > mat[2][maxs[2]] ? m : 2;

		int k = m;
		int l = maxs[m];
		float_t p = mat[k][l];

		// calculate cos and sin
		float_t y = (ret[l] - ret[k]) / 2.0f;
		float_t t = abs(y) + sqrt(p * p + y * y);
		float_t s = sqrt(p * p + t * t);
		float_t c = t / s;
		s = p / s;
		t = p * p / t;

		if (y < 0.0f) {
			s = -s;
			t = -t;
		}
		mat[k][l] = 0.0f;

		//update eigs
		ret[k] = ret[k] - t;
		ret[l] = ret[l] + t;

		// rotate
		for(int i = 0; i < k; i++) rotate(mat, c, s, i, k, i, l);
		for(int i = k + 1; i < l; i++) rotate(mat, c, s, k, i, i, l);
		for(int i = l + 1; i < 3; i++) rotate(mat, c, s, k, i, l, i);

		// update maximum index
		//if (k == 0 || l == 0)
		//	maxs[0] = mat[0][1] > mat[0][2] ? 1 : 2;

		int id1 = (k + 1) % 3;
		int id2 = (k + 2) % 3;
		maxs[k] = mat[k][id1] > mat[k][id2] ? id1 : id2;
		id1 = (l + 1) % 3;
		id2 = (l + 2) % 3;
		maxs[l] = mat[l][id1] > mat[l][id2] ? id1 : id2;
	}

	return ret;
}
#endif

