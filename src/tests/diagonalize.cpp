#include "engine/geometry/Math.h"
#include "common.h"

#include <iostream>

using namespace std;
using namespace glm;

int main(int argc, const char *argv[])
{
	int nTests, iters;
	mat3 mat;
	vec3 eigs;
	cin >> nTests;
	cin >> iters;
	while (nTests--) {
		REP(i, 3) {
			REP(j, 3) {
				cin >> mat[i][j];
			}
		}
		eigs = eigenvalues_jacobi(mat, iters);
		cout << eigs[0] << " " << eigs[1] << " " << eigs[2] << endl;
	}
	return 0;
}
