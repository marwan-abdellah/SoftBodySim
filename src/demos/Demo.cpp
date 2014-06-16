#define GLM_SWIZZLE

#include "common.h"

#include "GLFWApplication.h"
#include "engine/model/MeshData.h"
#include "engine/model/Material.h"
#include "engine/renderer/Renderer.h"
#include "engine/renderer/Camera.h"
#include "engine/SoftBody.h"
#include "engine/CUDASoftBodySolver.h"
#include "engine/solver/CPUSoftBodySolver.h"

#include <iostream>
#include <glm/gtc/constants.hpp>

using namespace std;

const double timestep = 0.02;
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
	Material mMat2;
	Material mMat3;
	bool cudaSolver;
	CUDASoftBodySolver::SoftBodyWorldParameters mWorldParams;
	Ray GetWoorldCoordinates(int x, int y);
	float_t mSpringness;
	bool mCameraMotion;
	SoftBody *b;
	bool mGrabb;
};

Demo::Demo(int argc, char **argv) :
	GLFWApplication("DemoApp", width, height),
	mCamera(glm::vec3(0,0,28), glm::vec3(0,-8.0f,0), glm::vec3(0,1,0)),
	mPaused(false),
	cudaSolver(0),
	mSpringness(0.9),
	mCameraMotion(false),
	b(0),
	mGrabb(0)
{
	const float_t groundLevel = -12.0;

	int res = mMat.LoadTextureFromBmp("src/demos/mrcrabs2.bmp");
	if (res) ERR("Texture loading failed!");
	md = MeshData::CreateFromObj("src/demos/crab.obj");
	md->material = &mMat;

	md1 = MeshData::CreateFromObj("src/demos/frog.obj");
	res = mMat2.LoadTextureFromBmp("src/demos/frog.bmp");
	if (res) ERR("Texture loading failed!");
	md1->material = &mMat2;

	md3 = MeshData::CreateFromObj("src/demos/Dino2.obj");
	res = mMat3.LoadTextureFromBmp("src/demos/Dino_512.bmp");
	if (res) ERR("Texture loading failed!");
	md3->material = &mMat3;

	md2 = MeshData::CreatePlane(50.0, 50.0, 2, 2);

	mWorldParams.gravity = glm::vec3(0, -10.0, 0);
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
	float delta = 0.1f;

	if (key == GLFW_KEY_LEFT_SHIFT) {
		if (action == GLFW_RELEASE)
			mCameraMotion = false;
		else if (action == GLFW_PRESS)
			mCameraMotion = true;
	}

	if (action == GLFW_RELEASE) return;

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
		b = new SoftBody(1, mSpringness, md1);
		mSolver->AddSoftBody(b);
	}
	if (key == GLFW_KEY_Y) {
		b = new SoftBody(1.0f, mSpringness, md);
		mSolver->AddSoftBody(b);
	}
	if (key == GLFW_KEY_U) {
		b = new SoftBody(1.0f, mSpringness, md3);
		mSolver->AddSoftBody(b);
	}
	if (key == GLFW_KEY_N)
		mSolver->ProjectSystem(timestep);
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
	if (key == GLFW_KEY_MINUS) {
		mSpringness -= 0.01;
		DBG("Springness : %f", mSpringness);
		softbodyList_t &list = mSolver->GetBodies();
		FOREACH_R(it, list)
			(*it)->SetSpringness(mSpringness);
	}
	if (key == GLFW_KEY_EQUAL) {
		mSpringness += 0.01;
		DBG("Springness : %f", mSpringness);
		softbodyList_t &list = mSolver->GetBodies();
		FOREACH_R(it, list)
			(*it)->SetSpringness(mSpringness);
	}
}

Ray Demo::GetWoorldCoordinates(int x, int y)
{
	glm::vec3 ret;
	// gl output coords
	// (-1,1)  ..... (1,1)
	//         .....
	// (-1,-1) ..... (1, -1)
	ret[0] = (2.0f * x) / width - 1.0f;
	ret[1] = 1.0f - (2.0 * y) / height;
	ret[2] = 1.0f; // place ray always in front of eye

	glm::vec4 pos = glm::vec4(ret, 1.0);
	pos = glm::inverse(renderer.GetProjectionMatrix()) * pos;
	pos[2] = -1.0f;
	pos[3] = 0.0f;

	pos = glm::inverse(mCamera.getCameraMatrix()) * pos;

	return Ray(mCamera.GetEyePosition(), pos.xyz());
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

	if (!mCameraMotion && state == GLFW_PRESS) { 
		Ray ray = GetWoorldCoordinates(x, y);
		mSolver->GrabStart(ray, 0.5f, 120.0f);
		mGrabb = true;
	}
	if (!mCameraMotion && state == GLFW_RELEASE) { 
		mSolver->GrabStop();
		mGrabb = false;
	}
}

void Demo::OnMouseMove(int x, int y)
{
	if (!mMousePressed)
		return;

	if (mCameraMotion) {
		static float angle = glm::pi<glm::float_t>() / (5.0f * 180.0f);
		int dx = x - mMouseLastX;
		int dy = y - mMouseLastY;

		glm::vec3 np;
		if (dx > 0)
			mCamera.moveRight(angle * dx);
		else
			mCamera.moveLeft(-angle * dx);
		if (dy > 0)
			mCamera.moveUp(angle * dy);
		else
			mCamera.moveDown(-angle * dy);
	}
	if (mGrabb) {
		Ray ray = GetWoorldCoordinates(x, y);
		mSolver->GrabUpdate(ray);
	}

	mMouseLastX = x;
	mMouseLastY = y;
}

int main(int argc, char **argv)
{
	Demo demo(argc, argv);
	demo.MainLoop(timestep);
}
