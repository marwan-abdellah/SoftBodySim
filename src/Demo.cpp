#include "SoftBody.h"
#include "GLUTApplcation.h"


class SoftBodyDemo : GLUTApplication {
	
	SoftBodyWorld world;
	const float_t dt;

	void initialize(void)
	{
		SoftBody a();
		a.loadFromFile('sadas')

		SoftBodySolver cpu_solver;
		SoftBodyRenderer renderer;

		SoftBodyWorld w(properties, cpu_solver, renderer);

		w.addBody(a);
	}

	void shutdown(void)
	{
	}

	void onDisplay(void)
	{
		w.time_proceed(dt);
		w.draw_all();
	}

	void onIdle(void)
	{
	}
};

int main(int argc, char **argv)
{
	SoftBodyDemo app(argc, argv);
	app.run();
}

