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
	SoftBodyRenderer    renderer;
	vector<Body*> mBodies;
	int mMouseLastX;
	int mMouseLastY;
	bool mMousePressed;
	Camera mCamera;
	SoftBodySolver *mSolver;
	bool mPaused;
	MeshData *md, *md1, *md2, *md3;
	Material mMat;
	bool cudaSolver;
	CUDASoftBodySolver::SoftBodyWorldParameters mWorldParams;
	float_t mSpringness;
};

Demo::Demo(int argc, char **argv) :
	GLFWApplication("DemoApp", width, height),
	mCamera(vec3(0,0,28), vec3(0,-8,0), vec3(0,1,0)),
	mPaused(false),
	cudaSolver(0),
	mSpringness(0.1)
{
	int res = mMat.LoadTextureFromBmp("src/demos/mrcrabs2.bmp");
	if (res) ERR("Texture loading failed!");
	const float_t groundLevel = -12.0;
	md = MeshData::CreateFromObj("src/demos/crab.obj");
	md->material = &mMat;
	md1 = MeshData::CreateFromObj("src/demos/cube_small.obj");
	md2 = MeshData::CreatePlane(50.0, 50.0, 2, 2);
	md3 = MeshData::CreatePlane(2.0, 2.0, 4, 4);

	mWorldParams.gravity = vec3(0, -10.0, 0);
	mWorldParams.leftWall = -15.0f;
	mWorldParams.rightWall = 15.0f;
	mWorldParams.backWall = -15.0f;
	mWorldParams.frontWall = 15.0f;
	mWorldParams.groundLevel = groundLevel;

	renderer.initialize(width, height);
	renderer.setRenderMethod(SB_RENDER_FACES);
	renderer.SetWorld(mWorldParams);

	mSolver = new CPUSoftBodySolver();
	mSolver->SetWorldParameters(mWorldParams);
	mSolver->Initialize();
}

Demo::~Demo(void)
{
	FOREACH(b, &mBodies)
		delete *b;
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
	renderer.DrawWorld(mCamera.getCameraMatrix());

	FOREACH_R(b, mSolver->GetBodies())
		renderer.renderBody(*b, mCamera.getCameraMatrix());
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
		mSolver->Shutdown();
		delete mSolver;
		cudaSolver = !cudaSolver;
		if (cudaSolver) {
			mSolver = new CUDASoftBodySolver();
			mSolver->SetWorldParameters(mWorldParams);
			DBG("CUDA Solver enabled");
		}
		else {
			mSolver = new CPUSoftBodySolver();
			mSolver->SetWorldParameters(mWorldParams);
			DBG("CPU Solver enabled");
		}
		mSolver->Initialize();
	}
	if (key == GLFW_KEY_C) {
		mSolver->Shutdown();
		mSolver->Initialize();
	}
	if (key == GLFW_KEY_T) {
		b = new SoftBody(1, mSpringness, 1.0,md1);
		b->SetColor(vec3(1.0, 1.0, 0.0f));
		mSolver->AddSoftBody(b);
	}
	if (key == GLFW_KEY_Y) {
		b = new SoftBody(1.0f, mSpringness, 1.0f, md);
		mSolver->AddSoftBody(b);
	}
	if (key == GLFW_KEY_U) {
		b = new SoftBody(1.0f, mSpringness, 1.0f, md3);
		b->SetColor(vec3(0.0, 0.0, 1.0f));
		mSolver->AddSoftBody(b);
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
	if (key == GLFW_KEY_MINUS) {
		mSpringness -= 0.001;
		DBG("Springness : %f", mSpringness);
		softbodyList_t &list = mSolver->GetBodies();
		FOREACH_R(it, list)
			(*it)->SetSpringness(mSpringness);
	}
	if (key == GLFW_KEY_EQUAL) {
		mSpringness += 0.001;
		DBG("Springness : %f", mSpringness);
		softbodyList_t &list = mSolver->GetBodies();
		FOREACH_R(it, list)
			(*it)->SetSpringness(mSpringness);
	}
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
