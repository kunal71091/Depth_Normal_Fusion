#include <GL/glew.h>

#include <d3d_glBase/glCommon.h>
#include "fishEyeDepthMapWarper.h"

#include <iostream>

using namespace D3D;

using std::make_pair;
using Eigen::Matrix;

template <typename T, typename U>
FishEyeDepthMapWarper<T, U>::FishEyeDepthMapWarper(int width, int height, U minDepth, U maxDepth, U xi, U maxDispDiff)
{
    this->width = width;
    this->height = height;
    this->minDepth = minDepth;
    this->maxDepth = maxDepth;
    this->maxDispDiff = maxDispDiff;
    this->xi = xi;

    count = new unsigned int;
    *count = 1;

    initializeGL();
}

template <typename T, typename U>
FishEyeDepthMapWarper<T,U>::FishEyeDepthMapWarper(const FishEyeDepthMapWarper<T,U>& other)
{
    this->width = other.width;
    this->height = other.height;
    this->minDepth = other.minDepth;
    this->maxDepth = other.maxDepth;
    this->maxDispDiff = other.maxDispDiff;
    this->xi = other.xi;


    this->fbo = other.fbo;
    this->depthbuffer = other.depthbuffer;
    this->colorbuffer = other.colorbuffer;
    this->fragShader = other.fragShader;
    this->vertShader = other.vertShader;
    this->shaderProgram = other.shaderProgram;

    this->count = other.count;
    (*(this->count))++;
}


template <typename T, typename U>
FishEyeDepthMapWarper<T,U>& FishEyeDepthMapWarper<T,U>::operator=(FishEyeDepthMapWarper<T,U>& other)
{
    // copy variables
    GLuint oldFbo = this->fbo;
    GLuint oldDepthbuffer = this->depthbuffer;
    GLuint oldColorbuffer = this->colorbuffer;
    GLuint oldFragShader = this->fragShader;
    GLuint oldVertShader = this->vertShader;
    GLuint oldShaderProgram = this->shaderProgram;
    unsigned int *oldCount = this->count;

    // assign new variables
    this->width = other.width;
    this->height = other.height;
    this->minDepth = other.minDepth;
    this->maxDepth = other.maxDepth;
    this->maxDispDiff = other.maxDispDiff;
    this->xi = other.xi;


    this->fbo = other.fbo;
    this->depthbuffer = other.depthbuffer;
    this->colorbuffer = other.colorbuffer;
    this->fragShader = other.fragShader;
    this->vertShader = other.vertShader;
    this->shaderProgram = other.shaderProgram;

    this->count = other.count;

    (*(this->count))++;

    // check if we need to delete
    if (--(*oldCount) == 0)
    {
        glDetachShader(oldShaderProgram, oldVertShader);
        glDetachShader(oldShaderProgram, oldFragShader);

        glDeleteShader(oldVertShader);
        glDeleteShader(oldFragShader);

        glDeleteProgram(oldShaderProgram);

        glDeleteRenderbuffersEXT(1, &oldColorbuffer);
        glDeleteRenderbuffersEXT(1, &oldDepthbuffer);

        glDeleteFramebuffersEXT(1, &oldFbo);

        delete oldCount;
    }

    D3D_GL_CHECK_ERROR

    return *this;
}


template <typename T, typename U>
FishEyeDepthMapWarper<T,U>::~FishEyeDepthMapWarper()
{
    if (--(*count) == 0)
    {
        glDetachShader(shaderProgram, vertShader);
        glDetachShader(shaderProgram, fragShader);

        glDeleteShader(vertShader);
        glDeleteShader(fragShader);

        glDeleteProgram(shaderProgram);

        glDeleteRenderbuffersEXT(1, &colorbuffer);
        glDeleteRenderbuffersEXT(1, &depthbuffer);

        glDeleteFramebuffersEXT(1, &fbo);

        delete count;
    }

    D3D_GL_CHECK_ERROR
}

template <typename T, typename U>
void FishEyeDepthMapWarper<T, U>::initializeGL()
{
    // initilaize GLEW
    D3D_GLEW_CHECKED_CALL( glewInit(); )

    D3D_GL_CHECK_ERROR

    // create a framebuffer object for offscreen rendering
    glGenFramebuffersEXT(1, &fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);

    // adding a depth buffer to the fbo
    glGenRenderbuffersEXT(1, &depthbuffer);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, width, height);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer);

    // colorbuffer
    glGenRenderbuffersEXT(1, &colorbuffer);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, colorbuffer);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB, width, height);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, colorbuffer);

    D3D_GL_CHECK_FBO(fbo)
    D3D_GL_CHECK_ERROR

    // create a vertex shader that calculates the depth
    const GLchar* vertShaderSrc[] = {
        "varying float depth;"
        "uniform float xi;"
        "void main()"
        "{"
        "   vec4 localPoint = gl_ModelViewMatrix*gl_Vertex;"
        "   depth = localPoint.z;"
        "   vec3 localPoint3;"
        "   localPoint3.x = localPoint.x/localPoint.w;"
        "   localPoint3.y = localPoint.y/localPoint.w;"
        "   localPoint3.z = localPoint.z/localPoint.w;"
        "   localPoint3 = localPoint3/length(localPoint3);"
        "   localPoint3.z = localPoint3.z + xi;"
        "   localPoint.x = depth*localPoint3.x/localPoint3.z;"
        "   localPoint.y = depth*localPoint3.y/localPoint3.z;"
        "   localPoint.z = depth;"
        "   localPoint.w = 1.0;"
        "   gl_Position = gl_ProjectionMatrix*localPoint;"
        "}"
    };

    vertShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vertShader, 1, vertShaderSrc, NULL);
    glCompileShader(vertShader);

    D3D_GL_CHECK_SHADER(vertShader)


    const GLchar* fragShaderSrc[] = {
        "varying float depth;"
        "uniform float maxDepth;"
        ""
        "void main()"
        "{"
        "   if (depth < 0.4)"
        "     discard;"
        "   gl_FragDepth = depth/maxDepth;"
        "}"
    };

    fragShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(fragShader, 1, fragShaderSrc, NULL);
    glCompileShader(fragShader);

    D3D_GL_CHECK_SHADER(fragShader)

    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, fragShader);
    glAttachShader(shaderProgram, vertShader);
    glLinkProgram(shaderProgram);

    D3D_GL_CHECK_PROGRAM(shaderProgram)

    // we can use the program after the check
    glUseProgram(shaderProgram);

    D3D_GL_CHECK_ERROR

    // set uniforms
    GLint maxDepthLoc;
    maxDepthLoc = glGetUniformLocation(shaderProgram, "maxDepth");
    D3D_GL_CHECK_ERROR
    glUniform1f(maxDepthLoc, maxDepth);

    GLint xiLoc;
    xiLoc = glGetUniformLocation(shaderProgram, "xi");
    D3D_GL_CHECK_ERROR
    glUniform1f(xiLoc, xi);

    D3D_GL_CHECK_ERROR

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);

    D3D_GL_CHECK_ERROR
}

template <typename T, typename U>
FishEyeDepthMap<T, U> FishEyeDepthMapWarper<T, U>::warpFishEyeDepthMap(const FishEyeDepthMap<T, U> &origFishEyeDepthMap, const FishEyeCameraMatrix<U> &newFishEyeCamera)
{
    // set up the projection
    setUpProjection(newFishEyeCamera);

    // set up the shaders
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glUseProgram(shaderProgram);

    // set and clear the buffers
    glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    //        glLoadIdentity();

    // draw triangles
    glBegin(GL_TRIANGLES);
    for (unsigned int y = 0; y < origFishEyeDepthMap.getHeight()-1; y++) {
        for (unsigned int x = 0; x < origFishEyeDepthMap.getWidth()-1; x++) {

            // get the four 3d points
            Matrix<U, 4, 1> point00 = origFishEyeDepthMap.unproject(x, y);
            Matrix<U, 4, 1> point01 = origFishEyeDepthMap.unproject(x+1, y);
            Matrix<U, 4, 1> point10 = origFishEyeDepthMap.unproject(x, y+1);
            Matrix<U, 4, 1> point11 = origFishEyeDepthMap.unproject(x+1, y+1);

            if(point00(3) == 0 || point01(3) == 0 || point10(3) == 0 || point11(3) == 0)
                continue;

            U dispDiff = std::max(std::max(std::fabs(1/origFishEyeDepthMap(x,y) - 1/origFishEyeDepthMap(x, y+1)), std::fabs(1/origFishEyeDepthMap(x, y+1)- 1/origFishEyeDepthMap(x+1,y))),std::fabs(1/origFishEyeDepthMap(x,y)- 1/origFishEyeDepthMap(x+1,y)));

            if (dispDiff < maxDispDiff && point00(3) > 0 && point01(3) > 0 && point10(3) > 0) {
                glVertex3f(point00(0), point00(1), point00(2));
                glVertex3f(point10(0), point10(1), point10(2));
                glVertex3f(point01(0), point01(1), point01(2));

                // project point to new camera on cpu for debugging
                //                    Vector3 point = newCamera.getP()*point01;
                //                    point /= point(2);

                //                    std::cout << point01(0) << " " << point01(1) << " " << point01(2) << std::endl;
                //                    std::cout << point(0) << " " << point(1) << std::endl << std::endl;
            }

            dispDiff = std::max(std::max(std::fabs(1/origFishEyeDepthMap(x+1,y) - 1/origFishEyeDepthMap(x, y+1)), std::fabs(1/origFishEyeDepthMap(x, y+1)- 1/origFishEyeDepthMap(x+1,y+1))),std::fabs(1/origFishEyeDepthMap(x+1,y)- 1/origFishEyeDepthMap(x+1,y+1)));

            if (dispDiff < maxDispDiff && point01(3) > 0 && point10(3) > 0 && point11(3) > 0) {
                glVertex3f(point01(0), point01(1), point01(2));
                glVertex3f(point10(0), point10(1), point10(2));
                glVertex3f(point11(0), point11(1), point11(2));
            }
        }
    }
    glEnd();
    glFlush();

    FishEyeDepthMap<T, U> warpedFishEyeDepthMap(width, height, newFishEyeCamera);

    // read back the depth map
    readResultBack(warpedFishEyeDepthMap.getDataPtr());

    D3D_GL_CHECK_ERROR

    // scale depth map back to right scale
    //warpedDepthMap *= maxDepth;

    return warpedFishEyeDepthMap;
}


template <typename T, typename U>
void FishEyeDepthMapWarper<T, U>::readResultBack(float* dataPtr)
{
    // glReadPixels(0,0,width,height, GL_RED, GL_FLOAT, dataPtr);
    glReadPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_FLOAT, dataPtr);

    for (float* ptr = dataPtr; ptr < dataPtr + width*height; ++ptr)
        *ptr = (*ptr == 1) ? 0.0f : *ptr * maxDepth;

}

template <typename T, typename U>
void FishEyeDepthMapWarper<T, U>::readResultBack(double* dataPtr)
{
    // glReadPixels(0,0,width, height, GL_RED, GL_DOUBLE, dataPtr);
    glReadPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_DOUBLE, dataPtr);

    for (double* ptr = dataPtr; ptr < dataPtr + width*height; ++ptr)
        *ptr = (*ptr == 1) ? 0.0f : *ptr * maxDepth;
}

template <typename T, typename U>
void FishEyeDepthMapWarper<T, U>::setUpProjection(const FishEyeCameraMatrix<U> &cam)
{
    // set projection
    glMatrixMode(GL_PROJECTION);

    // assumes that K(2,2) is 1
    const float fx = cam.getK()(0,0);
    const float fy = cam.getK()(1,1);
    const float px = cam.getK()(0,2);
    const float py = cam.getK()(1,2);

    float m[16];
    m[0] = 2*fx*(width-2)/(width*(width-1)); m[4] = 0;                                         m[8] = 2*px*(width-2)/(width*(width-1)) - 1 + 1/width;       m[12] = 0;
    m[1] = 0;                                m[5] = 2*fy*(height-2)/(height*(height-1));       m[9] = 2*py*(height-2)/(height*(height-1)) - 1 + 1/height;   m[13] = 0;
    m[2] = 0;                                m[6] = 0;                                         m[10] = (maxDepth+minDepth)/(maxDepth-minDepth);             m[14] = -(2*maxDepth*minDepth)/(maxDepth-minDepth);
    m[3] = 0;                                m[7] = 0;                                         m[11] = 1;                                                   m[15] = 0;

    // load the matrix
    glLoadMatrixf(m);

    // set model matrix
    glMatrixMode(GL_MODELVIEW);
    for (int i = 0; i < 15; i++)
        m[i] = 0;
    m[15] = 1;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m[i + j*4] = cam.getR()(i,j);
        }
        m[i + 12] = cam.getT()(i);
    }
//    for (int i = 0; i < 16; i++)
//        std::cout << m[i] << " ";
//    std::cout << std::endl;

    glLoadMatrixf(m);

    glViewport(0, 0, width, height);

    D3D_GL_CHECK_ERROR
}

template class FishEyeDepthMapWarper<float, float>;
template class FishEyeDepthMapWarper<float, double>;
template class FishEyeDepthMapWarper<double, float>;
template class FishEyeDepthMapWarper<double, double>;
