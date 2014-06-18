#ifndef SBS_RENDERER_RENREDER_H_
#define SBS_RENDERER_RENREDER_H_

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <glm/fwd.hpp>

#include <vector>
#include <list>

#include "Shader.h"
#include "sbs/model/SoftBody.h"
#include "sbs/solver/SoftBodySolver.h"

typedef enum SoftBodyRenderMethod {
    SB_RENDER_FACES,
    SB_RENDER_PARTICLES,
} SoftBodyRenderMethod_e;

class SoftBodyRenderer {

    public:
		SoftBodyRenderer();
        void                            initialize(int w, int h);
        void                            shutdown(void);
        void                            setRenderMethod(SoftBodyRenderMethod_e);
        SoftBodyRenderMethod_e          getRenderMethod(void);
        void                            renderBody(Body *s, const glm::mat4 &view);
        void                            clearScreen(void);
        void                            setLightSource(glm::vec3);
		void     SetWorld(SoftBodySolver::SoftBodyWorldParameters &params);
		void DrawWorld(const glm::mat4 &view);
		glm::mat4 &GetProjectionMatrix(void) { return mProjectionMat; }

    private:
        void logShader(GLint shader);
        int                             mWidth, mHeight;
        SoftBodyRenderMethod_e          mMethod;
        glm::mat4                       mProjectionMat;
        Shader                          *mCurrent;
        Shader                          mLighting;
        Shader                          mGround;
        Shader                          mPointLine;
		VertexBuffer                    *mWorld;
		VertexBuffer                    *mSphere;
};

#endif
