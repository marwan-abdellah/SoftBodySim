#include "engine/geometry/Math.h"
#include "common.h"

#include <iostream>
#include <glm/ext.hpp>

using namespace std;
using namespace glm;

static int print(mat3 &mat)
{
	for(int i =0; i < 3; i++)
		ERR("%f %f %f", mat[i][0], mat[i][1], mat[i][2]);
}

int main(int argc, const char *argv[])
{
	int nTests, iters;
	mat3 mat, E;
	vec3 eigs;
	cin >> nTests;
	cin >> iters;
	while (nTests--) {
		REP(i, 3) {
			REP(j, 3) {
				cin >> mat[i][j];
			}
		}
		cout.precision(5);
		mat3 t = mat;
		eigs = eigenvalues_jacobi(t, iters, E);
		mat3 D = diagonal3x3(eigs);
		D[0][0] = sqrt(D[0][0]);
		D[1][1] = sqrt(D[1][1]);
		D[2][2] = sqrt(D[2][2]);
		mat3 O = inverse(E) * D *E;
		O = O * O;
		bool equal = true;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				if (abs(O[i][j] - mat[i][j]) > 0.0001f)
					equal = false;
		cout << equal << endl;
		if (!equal) {
			print(D);
			print(mat);
			ERR("%f %f %f", eigs[0], eigs[1], eigs[2]);
			print(E);
			print(O);
		}
	}
	return 0;
}
