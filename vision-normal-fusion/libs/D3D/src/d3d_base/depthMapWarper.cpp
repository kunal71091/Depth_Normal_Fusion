#include <GL/glew.h>

#include <d3d_glBase/glCommon.h>
#include "depthMapWarper.h"

#include <iostream>

using namespace D3D;

using std::make_pair;
using Eigen::Matrix;

template <typename T, typename U>
DepthMapWarper<T, U>::DepthMapWarper(int width, int height, U minDepth, U maxDepth, U maxDispDiff)
{
    this->width = width;
    this->height = height;
    this->minDepth = minDepth;
    this->maxDepth = maxDepth;
    this->maxDispDiff = maxDispDiff;

    count = new unsigned int;
    *count = 1;

    initializeGL();
}

template <typename T, typename U>
DepthMapWarper<T,U>::DepthMapWarper(const DepthMapWarper<T,U>& other)
{
    this->width = other.width;
    this->height = other.height;
    this->minDepth = other.minDepth;
    this->maxDepth = other.maxDepth;
    this->maxDispDiff = other.maxDispDiff;

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
DepthMapWarper<T,U>& DepthMapWarper<T,U>::operator=(DepthMapWarper<T,U>& other)
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
DepthMapWarper<T, U>::~DepthMapWarper()
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
    }

    D3D_GL_CHECK_ERROR
}

template <typename T, typename U>
void DepthMapWarper<T, U>::initializeGL()
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
        "varying float attachedVal1;"
        "varying float attachedVal2;"
        "void main()"
        "{"
        "   gl_Position = ftransform();"
        "   depth = (gl_ModelViewMatrix * gl_Vertex).z;"
        "   attachedVal1 = gl_Color.g;"
        "   attachedVal2 = gl_Color.b;"
        "}"
    };

    vertShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vertShader, 1, vertShaderSrc, NULL);
    glCompileShader(vertShader);

    D3D_GL_CHECK_SHADER(vertShader)


    // create a fragment shader that stores the depth value in the red channel

    const GLchar* fragShaderSrc[] = {
        "varying float depth;"
        "varying float attachedVal1;"
        "varying float attachedVal2;"
        "uniform float maxDepth;"
        "void main()"
        "{"
        "   gl_FragDepth = depth/maxDepth;"
      //"   gl_FragColor.r = depth/maxDepth;"
        "   gl_FragColor.g = attachedVal1;"
        "   gl_FragColor.b = attachedVal2;"
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

    D3D_GL_CHECK_ERROR

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);

    D3D_GL_CHECK_ERROR
}

template <typename T, typename U>
DepthMap<T, U> DepthMapWarper<T, U>::warpDepthMap(const DepthMap<T, U> &origDepthMap, const CameraMatrix<U> &newCamera)
{
    // set up the projection
    setUpProjection(newCamera);

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
    for (unsigned int y = 0; y < origDepthMap.getHeight()-1; y++) {
        for (unsigned int x = 0; x < origDepthMap.getWidth()-1; x++) {

            // get the four 3d points
            Matrix<U, 4, 1> point00 = origDepthMap.unproject(x, y);
            Matrix<U, 4, 1> point01 = origDepthMap.unproject(x+1, y);
            Matrix<U, 4, 1> point10 = origDepthMap.unproject(x, y+1);
            Matrix<U, 4, 1> point11 = origDepthMap.unproject(x+1, y+1);

            U dispDiff = std::max(std::max(std::fabs(1/origDepthMap(x,y) - 1/origDepthMap(x, y+1)), std::fabs(1/origDepthMap(x, y+1)- 1/origDepthMap(x+1,y))),std::fabs(1/origDepthMap(x,y)- 1/origDepthMap(x+1,y)));

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

            dispDiff = std::max(std::max(std::fabs(1/origDepthMap(x+1,y) - 1/origDepthMap(x, y+1)), std::fabs(1/origDepthMap(x, y+1)- 1/origDepthMap(x+1,y+1))),std::fabs(1/origDepthMap(x+1,y)- 1/origDepthMap(x+1,y+1)));

            if (dispDiff < maxDispDiff && point01(3) > 0 && point10(3) > 0 && point11(3) > 0) {
                glVertex3f(point01(0), point01(1), point01(2));
                glVertex3f(point10(0), point10(1), point10(2));
                glVertex3f(point11(0), point11(1), point11(2));
            }
        }
    }
    glEnd();
    glFlush();

    DepthMap<T, U> warpedDepthMap(width, height, newCamera);

    // read back the depth map
    readResultBack(warpedDepthMap.getDataPtr());

    D3D_GL_CHECK_ERROR

    // scale depth map back to right scale
    //warpedDepthMap *= maxDepth;

    return warpedDepthMap;
}

template <typename T, typename U>
pair<DepthMap<T,U>, Grid<T> > DepthMapWarper<T,U>::warpDepthMap(const DepthMap<T, U>& origDepthMap, const Grid<T>& attachedVal, const CameraMatrix<U>& newCamera)
{
    // set up the projection
    setUpProjection(newCamera);

    // set and clear the buffers
    glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    //        glLoadIdentity();

    // draw triangles
    glBegin(GL_TRIANGLES);
    for (unsigned int y = 0; y < origDepthMap.getHeight()-1; y++) {
        for (unsigned int x = 0; x < origDepthMap.getWidth()-1; x++) {

            // get the four 3d points
            Matrix<U, 4, 1> point00 = origDepthMap.unproject(x, y);
            Matrix<U, 4, 1> point01 = origDepthMap.unproject(x+1, y);
            Matrix<U, 4, 1> point10 = origDepthMap.unproject(x, y+1);
            Matrix<U, 4, 1> point11 = origDepthMap.unproject(x+1, y+1);

            U dispDiff = std::max(std::max(std::fabs(1/origDepthMap(x,y) - 1/origDepthMap(x, y+1)), std::fabs(1/origDepthMap(x, y+1)- 1/origDepthMap(x+1,y))),std::fabs(1/origDepthMap(x,y)- 1/origDepthMap(x+1,y)));

            if (dispDiff < maxDispDiff && point00(3) > 0 && point01(3) > 0 && point10(3) > 0) {
                glColor3f(0.0, attachedVal(x,y), 0.0);
                glVertex3f(point00(0), point00(1), point00(2));
                glColor3f(0.0, attachedVal(x,y+1), 0.0);
                glVertex3f(point10(0), point10(1), point10(2));
                glColor3f(0.0, attachedVal(x+1,y), 0.0);
                glVertex3f(point01(0), point01(1), point01(2));

                // project point to new camera on cpu for debugging
                //                    Vector3 point = newCamera.getP()*point01;
                //                    point /= point(2);

                //                    std::cout << point01(0) << " " << point01(1) << " " << point01(2) << std::endl;
                //                    std::cout << point(0) << " " << point(1) << std::endl << std::endl;
            }

            dispDiff = std::max(std::max(std::fabs(1/origDepthMap(x+1,y) - 1/origDepthMap(x, y+1)), std::fabs(1/origDepthMap(x, y+1)- 1/origDepthMap(x+1,y+1))),std::fabs(1/origDepthMap(x+1,y)- 1/origDepthMap(x+1,y+1)));

            if (dispDiff < maxDispDiff && point01(3) > 0 && point10(3) > 0 && point11(3) > 0) {
                glColor3f(0.0, attachedVal(x+1,y), 0.0);
                glVertex3f(point01(0), point01(1), point01(2));
                glColor3f(0.0, attachedVal(x,y+1), 0.0);
                glVertex3f(point10(0), point10(1), point10(2));
                glColor3f(0.0, attachedVal(x+1,y+1), 0.0);
                glVertex3f(point11(0), point11(1), point11(2));
            }
        }
    }
    glEnd();
    glFlush();

    // read back the depth map
    DepthMap<T, U> warpedDepthMap(width, height, newCamera);
    readResultBack(warpedDepthMap.getDataPtr());
    D3D_GL_CHECK_ERROR

    // scale depth map back to right scale
    //warpedDepthMap *= maxDepth;

    // read attached values back
    Grid<T> warpedAttachedVal(width, height);
    readAttachedVal1Back(warpedAttachedVal.getDataPtr());
    D3D_GL_CHECK_ERROR

    return make_pair(warpedDepthMap, warpedAttachedVal);
}


template <typename T, typename U>
pair<DepthMap<T,U>, pair<Grid<T>, Grid<T> > > DepthMapWarper<T,U>::warpDepthMap(const DepthMap<T, U>& origDepthMap, const Grid<T>& attachedVal1, const Grid<T>& attachedVal2, const CameraMatrix<U>& newCamera)
{
    // set up the projection
    setUpProjection(newCamera);

    // set and clear the buffers
    glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    //        glLoadIdentity();

    // draw triangles
    glBegin(GL_TRIANGLES);
    for (unsigned int y = 0; y < origDepthMap.getHeight()-1; y++) {
        for (unsigned int x = 0; x < origDepthMap.getWidth()-1; x++) {

            // get the four 3d points
            Matrix<U, 4, 1> point00 = origDepthMap.unproject(x, y);
            Matrix<U, 4, 1> point01 = origDepthMap.unproject(x+1, y);
            Matrix<U, 4, 1> point10 = origDepthMap.unproject(x, y+1);
            Matrix<U, 4, 1> point11 = origDepthMap.unproject(x+1, y+1);

            if(point00(3) == 0 || point01(3) == 0 || point10(3) == 0 || point11(3) == 0)
                continue;

            U dispDiff = std::max(std::max(std::fabs(1/origDepthMap(x,y) - 1/origDepthMap(x, y+1)), std::fabs(1/origDepthMap(x, y+1)- 1/origDepthMap(x+1,y))),std::fabs(1/origDepthMap(x,y)- 1/origDepthMap(x+1,y)));

            if (dispDiff < maxDispDiff && point00(3) > 0 && point01(3) > 0 && point10(3) > 0) {
                glColor3f(0.0, attachedVal1(x,y), attachedVal2(x,y));
                glVertex3f(point00(0), point00(1), point00(2));
                glColor3f(0.0, attachedVal1(x,y+1), attachedVal2(x,y+1));
                glVertex3f(point10(0), point10(1), point10(2));
                glColor3f(0.0, attachedVal1(x+1,y), attachedVal2(x+1,y));
                glVertex3f(point01(0), point01(1), point01(2));
            }

            dispDiff = std::max(std::max(std::fabs(1/origDepthMap(x+1,y) - 1/origDepthMap(x, y+1)), std::fabs(1/origDepthMap(x, y+1)- 1/origDepthMap(x+1,y+1))),std::fabs(1/origDepthMap(x+1,y)- 1/origDepthMap(x+1,y+1)));

            if (dispDiff < maxDispDiff && point01(3) > 0 && point10(3) > 0 && point11(3) > 0) {
                glColor3f(0.0, attachedVal1(x+1,y), attachedVal2(x+1,y));
                glVertex3f(point01(0), point01(1), point01(2));
                glColor3f(0.0, attachedVal1(x,y+1), attachedVal2(x,y+1));
                glVertex3f(point10(0), point10(1), point10(2));
                glColor3f(0.0, attachedVal1(x+1,y+1), attachedVal2(x+1,y+1));
                glVertex3f(point11(0), point11(1), point11(2));
            }
        }
    }
    glEnd();
    glFlush();

    // read back the depth map
    DepthMap<T, U> warpedDepthMap(width, height, newCamera);
    readResultBack(warpedDepthMap.getDataPtr());
    D3D_GL_CHECK_ERROR

    // scale depth map back to right scale
    //warpedDepthMap *= maxDepth;

    // read attached values back
    Grid<T> warpedAttachedVal1(width, height);
    readAttachedVal1Back(warpedAttachedVal1.getDataPtr());
    D3D_GL_CHECK_ERROR


    Grid<T> warpedAttachedVal2(width, height);
    readAttachedVal2Back(warpedAttachedVal2.getDataPtr());
    D3D_GL_CHECK_ERROR

    return make_pair(warpedDepthMap, make_pair(warpedAttachedVal1, warpedAttachedVal2));
}


template <typename T, typename U>
void DepthMapWarper<T, U>::readResultBack(float* dataPtr)
{
    // glReadPixels(0,0,width,height, GL_RED, GL_FLOAT, dataPtr);
    glReadPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_FLOAT, dataPtr);

    for (float* ptr = dataPtr; ptr < dataPtr + width*height; ++ptr)
        *ptr = (*ptr == 1) ? 0.0f : *ptr * maxDepth;

}

template <typename T, typename U>
void DepthMapWarper<T, U>::readResultBack(double* dataPtr)
{
    // glReadPixels(0,0,width, height, GL_RED, GL_DOUBLE, dataPtr);
    glReadPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_DOUBLE, dataPtr);

    for (double* ptr = dataPtr; ptr < dataPtr + width*height; ++ptr)
        *ptr = (*ptr == 1) ? 0.0f : *ptr * maxDepth;
}

template <typename T, typename U>
void DepthMapWarper<T, U>::readAttachedVal1Back(float* dataPtr)
{
    glReadPixels(0,0,width,height, GL_GREEN, GL_FLOAT, dataPtr);
}

template <typename T, typename U>
void DepthMapWarper<T, U>::readAttachedVal1Back(double* dataPtr)
{
    glReadPixels(0,0, width, height, GL_GREEN, GL_DOUBLE, dataPtr);
}

template <typename T, typename U>
void DepthMapWarper<T, U>::readAttachedVal2Back(float* dataPtr)
{
    glReadPixels(0,0,width,height, GL_BLUE, GL_FLOAT, dataPtr);
}

template <typename T, typename U>
void DepthMapWarper<T, U>::readAttachedVal2Back(double* dataPtr)
{
    glReadPixels(0,0, width, height, GL_BLUE, GL_DOUBLE, dataPtr);
}

template <typename T, typename U>
void DepthMapWarper<T, U>::setUpProjection(const CameraMatrix<U> &cam)
{
    // set projection
    glMatrixMode(GL_PROJECTION);

    // assumes that K(2,2) is 1
    const float fx = cam.getK()(0,0);
    const float fy = cam.getK()(1,1);
    const float px = cam.getK()(0,2);
    const float py = cam.getK()(1,2);

    float m[16];
//    m[0] = 2*fx*(width-2)/(width*(width-1)); m[4] = 0;                                         m[8] = 2*px*(width-2)/(width*(width-1)) - 1 + 1/width;       m[12] = 0;
//    m[1] = 0;                                m[5] = 2*fy*(height-2)/(height*(height-1));       m[9] = 2*py*(height-2)/(height*(height-1)) - 1 + 1/height;   m[13] = 0;
//    m[2] = 0;                                m[6] = 0;                                         m[10] = (maxDepth+minDepth)/(maxDepth-minDepth);             m[14] = -(2*maxDepth*minDepth)/(maxDepth-minDepth);
//    m[3] = 0;                                m[7] = 0;                                         m[11] = 1;                                                   m[15] = 0;

    // version from Thomas, probably correct
    m[0] = (2*fx)/width;   m[4] = 0;               m[8] = 2*(0.5f + px)/width - 1.0f;                 m[12] = 0;
    m[1] = 0;              m[5] = (2*fy)/height;   m[9] = 2*(0.5f + py)/height - 1.0f;                m[13] = 0;
    m[2] = 0;              m[6] = 0;               m[10] = (maxDepth+minDepth)/(maxDepth-minDepth);   m[14] = -(2*maxDepth*minDepth)/(maxDepth-minDepth);
    m[3] = 0;              m[7] = 0;               m[11] = 1;                                         m[15] = 0;

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

template class DepthMapWarper<float, float>;
template class DepthMapWarper<float, double>;
template class DepthMapWarper<double, float>;
template class DepthMapWarper<double, double>;
