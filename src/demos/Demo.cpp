#include "common.h"
#include "GLFWApplication.h"
#include "engine/model/MeshData.h"
#include "engine/model/Material.h"
#include "renderer/Renderer.h"
#include "renderer/Camera.h"
#include "engine/SoftBody.h"
#include "engine/CUDASoftBodySolver.h"
#include "engine/solver/CPUSoftBodySolver.h"

#include <iostream>
#include <glm/glm.hpp>

using namespace std;
using namespace glm;

#define SIZE(x) (sizeof(x)/sizeof(x[0]))

const int width = 800;
const int height = 600;


class Demo : public GLFWApplication
{
public:
	Demo(int argc, char **argv);
	~Demo(void);
	void OnKeyboard(int k, int action);
	void OnMouseClick(int type, int state, int x, int y);
	void OnMouseMove(int x, int y);
	void OnRender(void);
	void OnUpdate(double dt);
private:
	Body *floor;
	SoftBodyRenderer    renderer;
	softbodyList_t     mSoftBodies;
	int mMouseLastX;
	int mMouseLastY;
	bool mMousePressed;
	Camera mCamera;
	SoftBodySolver *mSolver;
	int	mEnginUpdateTime;
	mat4 mFloorTransform;
	bool mPaused;
	MeshData *md, *md1, *md2, *md3;
	Material mMat;
	bool cudaSolver;
};

Demo::Demo(int argc, char **argv) :
	GLFWApplication("DemoApp", width, height),
	mCamera(vec3(0,0,8), vec3(0,0,0), vec3(0,1,0)),
	mPaused(false),
	cudaSolver(0)
{
	int res = mMat.LoadTextureFromBmp("src/demos/mrcrabs2.bmp");
	if (res) ERR("Texture loading failed!");
	const float_t groundLevel = -2.0;
	md = MeshData::CreateFromObj("src/demos/crab.obj");
	md1 = MeshData::CreateFromObj("src/demos/cube_small.obj");
	md2 = MeshData::CreatePlane(300.0, 300.0, 2, 2);
	md3 = MeshData::CreatePlane(2.0, 2.0, 4, 4);

	md->material = &mMat;

	/*
	ERR("Mesh trianges: %d", md->nodesTriangles.size());
	ERR("Mesh vertexes: %d", md->vertexes.size());

	index3Array_t::iterator it2 = md->faces.begin();
	FOREACH(it, &md->nodesTriangles) {
		ERR("(%d %d %d) - (%d %d %d)", (*it)[0], (*it)[1], (*it)[2],
		(*it2)[0], (*it2)[1], (*it2)[2]); 
		it2++;
	}
	MeshData::vertexArray_t::iterator vit = md->vertexes.begin();
	FOREACH(it, &md->nodes) {
		ERR("(%f %f %f) - (%f %f %f)[%d]", (*it)[0], (*it)[1], (*it)[2],
				vit->position[0], vit->position[1], vit->position[2],
				md->vertexesNodes[std::distance(md->vertexes.begin(), vit)]);
		vit++;
	}
	*/

	floor = new Body(md2);
	mFloorTransform = translate(0.0f, groundLevel - 0.001f, 0.0f); // add delta to avoid z-fighting
	mFloorTransform = rotate(mFloorTransform, -90.0f, 1.0f, 0.0f, 0.0f);

	renderer.initialize(width, height);
	renderer.setRenderMethod(SB_RENDER_FACES);

	CUDASoftBodySolver::SoftBodyWorldParameters worldParams;
	worldParams.gravity = vec3(0, -10.0, 0);
	worldParams.groundLevel = groundLevel;

	mSolver = new CPUSoftBodySolver();
	mSolver->SetWorldParameters(worldParams);
	mSolver->Initialize();
}

Demo::~Demo(void)
{
	FOREACH(b, &mSoftBodies)
		delete *b;
	delete floor;
	delete md;
	delete md1;
	delete md2;
	delete md3;
	delete mSolver;
}

void Demo::OnUpdate(double dt)
{
	if (mPaused) return;
	mSolver->ProjectSystem(dt);
}

void Demo::OnRender(void)
{
	mSolver->UpdateVertexBuffers();

	renderer.clearScreen();
	FOREACH(b, &mSoftBodies)
		renderer.renderBody(*b, mCamera.getCameraMatrix());

	renderer.renderBody(floor, mCamera.getCameraMatrix() * mFloorTransform);
}

void Demo::OnKeyboard(int key, int action)
{
	float angle = 2.0f;
	float delta = 0.1f;
	SoftBody *b;

	if (action == GLFW_RELEASE) return;

	if (key == GLFW_KEY_W)
		mCamera.moveUp(angle);
	if (key == GLFW_KEY_D)
		mCamera.moveDown(angle);
	if (key == GLFW_KEY_Z)
		mCamera.moveIn(delta);
	if (key == GLFW_KEY_X)
		mCamera.moveOut(delta);
	if (key == GLFW_KEY_P)
		mPaused = !mPaused;
	if (key == GLFW_KEY_V) {
		mSoftBodies.clear();
		delete mSolver;
		cudaSolver = !cudaSolver;
		if (cudaSolver) {
			mSolver = new CUDASoftBodySolver();
			DBG("CUDA Solver enabled");
		}
		else {
			mSolver = new CPUSoftBodySolver();
			DBG("CPU Solver enabled");
		}
		mSolver->Initialize();
	}
	if (key == GLFW_KEY_C) {
		mSoftBodies.clear();
		mSolver->Shutdown();
		mSolver->Initialize();
	}
	if (key == GLFW_KEY_T) {
		b = new SoftBody(1,0.1,1,md1);
		b->SetColor(vec3(1.0, 1.0, 0.0f));
		mSolver->AddSoftBody(b);
		mSoftBodies.push_back(b);
	}
	if (key == GLFW_KEY_Y) {
		b = new SoftBody(1.0f, 0.1f, 1.0f, md);
		b->SetColor(vec3(0.0, 0.0, 1.0f));
		mSolver->AddSoftBody(b);
		mSoftBodies.push_back(b);
	}
	if (key == GLFW_KEY_U) {
		b = new SoftBody(1.0f, 0.1f, 1.0f, md3);
		b->SetColor(vec3(0.0, 0.0, 1.0f));
		mSolver->AddSoftBody(b);
		mSoftBodies.push_back(b);
	}
	if (key == GLFW_KEY_N)
		mSolver->ProjectSystem(0.02);
	if (key == GLFW_KEY_M)
	{
		switch (renderer.getRenderMethod()) {
			case SB_RENDER_PARTICLES:
				renderer.setRenderMethod(SB_RENDER_FACES);
				break;
			case SB_RENDER_FACES:
				renderer.setRenderMethod(SB_RENDER_PARTICLES);
			break;
		}
	}
	if (key == GLFW_KEY_A)
		mCamera.moveRight(angle);
	else if (key == GLFW_KEY_D)
		mCamera.moveLeft(angle);
}

void Demo::OnMouseClick(int type, int state, int x, int y)
{
	if (type != GLFW_MOUSE_BUTTON_1)
		return;

	if (state == GLFW_PRESS) {
		mMouseLastX = x;
		mMouseLastY = y;
		mMousePressed = true;
	}
	else if (state == GLFW_RELEASE)
		mMousePressed = false;
}

void Demo::OnMouseMove(int x, int y)
{
	if (!mMousePressed)
		return;

	static float angle = 0.2f;
	int dx = x - mMouseLastX;
	int dy = y - mMouseLastY;

	vec3 np;
	if (dx > 0)
		mCamera.moveRight(angle * dx);
	else
		mCamera.moveLeft(-angle * dx);
	if (dy > 0)
		mCamera.moveUp(angle * dy);
	else
		mCamera.moveDown(-angle * dy);

	mMouseLastX = x;
	mMouseLastY = y;
}

int main(int argc, char **argv)
{
	Demo demo(argc, argv);
	demo.MainLoop(0.02);
}
