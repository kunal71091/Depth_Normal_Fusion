#include "cameraPoseGrouper.h"
#include <d3d_glBase/glCommon.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace D3D
{

using namespace D3D_CUDA;

CameraPoseGrouper::CameraPoseGrouper()
{
    numPlanes = 10;
    minZ = 1;
    maxZ = 10;
    scale = 1;
    width = 100;
    height = 100;
    initialized = false;

    fbo = 0;
    depthbuffer = 0;
    colorbuffer = 0;
    fragShader = 0;
    vertShader = 0;
    shaderProgram = 0;

    bufRes = 0;
    dArray = 0;
    rowSums = 0;
}

CameraPoseGrouper::~CameraPoseGrouper()
{
    cleanUpGLCuda();
}

void CameraPoseGrouper::setViewingRange(float minZ, float maxZ)
{
    this->minZ = minZ;
    this->maxZ = maxZ;
}

void CameraPoseGrouper::setNumPlanes(int numPlanes)
{
    this->numPlanes = numPlanes;
}

void CameraPoseGrouper::setScale(float scale)
{
    if (cameras.size() > 0)
        D3D_THROW_EXCEPTION("Changing the scaling after adding cameras is not possible.")

    if (this->scale != scale)
    {
        this->scale = scale;
        initialized = false;
    }
}

void CameraPoseGrouper::setSize(int width, int height)
{
    if (this->width != width || this->height != height)
    {
        initialized = false;
        this->width = width;
        this->height = height;
    }
}

void CameraPoseGrouper::cleanUpGLCuda()
{
    CameraPoseGrouperDeviceCode::cleanUpCudaInterop(&bufRes, &rowSums);

    if (shaderProgram != 0)
    {
        glDeleteProgram(shaderProgram);
        shaderProgram = 0;
        D3D_GL_CHECK_ERROR
    }

    if (vertShader != 0)
    {
        glDeleteShader(vertShader);
        vertShader = 0;
        D3D_GL_CHECK_ERROR
    }

    if (fragShader != 0)
    {
        glDeleteShader(fragShader);
        fragShader = 0;
        D3D_GL_CHECK_ERROR
    }

    if (depthbuffer != 0)
    {
        glDeleteRenderbuffersEXT(1, &depthbuffer);
        depthbuffer = 0;
        D3D_GL_CHECK_ERROR
    }

    if (colorbuffer != 0)
    {
        glDeleteRenderbuffersEXT(1, &colorbuffer);
        colorbuffer = 0;
        D3D_GL_CHECK_ERROR
    }

    if (fbo != 0)
    {
        glDeleteFramebuffersEXT(1, &fbo);
        fbo = 0;
        D3D_GL_CHECK_ERROR
    }


}

void CameraPoseGrouper::initializeGLCuda()
{
    // try to clean up old stuff
    cleanUpGLCuda();


    // scaled width and height
    widthScaled = round(scale*width);
    heightScaled = round(scale*height);

    // initilaize GLEW
    D3D_GLEW_CHECKED_CALL( glewInit(); )

    D3D_GL_CHECK_ERROR

    // create a framebuffer object for offscreen rendering
    glGenFramebuffersEXT(1, &fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);

    // adding a depth buffer to the fbo
    glGenRenderbuffersEXT(1, &depthbuffer);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, widthScaled, heightScaled);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer);

    // colorbuffer
    glGenRenderbuffersEXT(1, &colorbuffer);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, colorbuffer);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RED, widthScaled, heightScaled);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, colorbuffer);

    D3D_GL_CHECK_FBO(fbo)
    D3D_GL_CHECK_ERROR

    // create a vertex shader that calculates the depth
    const GLchar* vertShaderSrc[] = {
        "varying float depth;"
        "void main()"
        "{"
        "   gl_Position = ftransform();"
        "}"
    };

    vertShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vertShader, 1, vertShaderSrc, NULL);
    glCompileShader(vertShader);

    D3D_GL_CHECK_SHADER(vertShader)


    // create a fragment shader that stores the depth value in the red channel

    const GLchar* fragShaderSrc[] = {
            "void main()"
            "{"
            "   gl_FragColor.r = 1.0;"
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

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);

    D3D_GL_CHECK_ERROR

    CameraPoseGrouperDeviceCode::setUpCudaInterop(colorbuffer, &bufRes, &dArray, &rowSums);

    initialized = true;
}

void CameraPoseGrouper::addCamera(int id, const D3D::CameraMatrix<double> &cam)
{
    D3D::CameraMatrix<double> scaled = cam;
    scaled.scaleK(scale, scale);
    cameras[id] = scaled;
}

void CameraPoseGrouper::computeOverlaps(int refCamId, std::vector<std::pair<int, float> > &overlaps)
{
    if (!initialized)
        initializeGLCuda();

    if (cameras.count(refCamId) != 1)
    {
        std::stringstream strstream;
        strstream << "Camera with ID " << refCamId << " does not exist.";
        D3D_THROW_EXCEPTION(strstream.str().c_str());
    }

    D3D::CameraMatrix<double> refCam = cameras[refCamId];

    // set up the projection
    // set up the projection to the reference camera
    // set projection
    glMatrixMode(GL_PROJECTION);

    // assumes that K(2,2) is 1
    const float fx = refCam.getK()(0,0);
    const float fy = refCam.getK()(1,1);
    const float px = refCam.getK()(0,2);
    const float py = refCam.getK()(1,2);

    // avoid floating point problems with first and last plane
    // was maybe a stupid idea
    float m[16];
    m[0] = 2*fx*(widthScaled-2)/(widthScaled*(widthScaled-1)); m[4] = 0;                                         m[8] = 2*px*(widthScaled-2)/(widthScaled*(widthScaled-1)) - 1 + 1/widthScaled;       m[12] = 0;
    m[1] = 0;                                m[5] = 2*fy*(heightScaled-2)/(heightScaled*(heightScaled-1));       m[9] = 2*py*(heightScaled-2)/(heightScaled*(heightScaled-1)) - 1 + 1/heightScaled;   m[13] = 0;
    m[2] = 0;                                m[6] = 0;                                         m[10] = (maxZ+minZ)/(maxZ-minZ);                             m[14] = -(2*maxZ*minZ)/(maxZ-minZ);
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
            m[i + j*4] = refCam.getR()(i,j);
        }
        m[i + 12] = refCam.getT()(i);
    }

    glLoadMatrixf(m);

    glViewport(0, 0, widthScaled, heightScaled);

    D3D_GL_CHECK_ERROR

    for (std::map<int, D3D::CameraMatrix<double> >::iterator it = cameras.begin(); it != cameras.end(); it++)
    {
        if (it->first == refCamId)
            continue;

        float overlap = 0;

        for (int i = 0; i < numPlanes; i++)
        {
            double z = minZ + i*(maxZ - minZ)/(numPlanes-1);

            Eigen::Vector4d topLeft = it->second.unprojectPoint(-0.5, -0.5, z);
            Eigen::Vector4d topRight = it->second.unprojectPoint(widthScaled+1, -0.5, z);
            Eigen::Vector4d bottomRight = it->second.unprojectPoint(widthScaled+1, heightScaled+1, z);
            Eigen::Vector4d bottomLeft = it->second.unprojectPoint(-0.5, heightScaled+1, z);

            // set and clear the buffers
            glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // draw plane
            glBegin(GL_QUADS);
            glVertex3d(topLeft(0), topLeft(1), topLeft(2));
            glVertex3d(topRight(0), topRight(1), topRight(2));
            glVertex3d(bottomRight(0), bottomRight(1), bottomRight(2));
            glVertex3d(bottomLeft(0), bottomLeft(1), bottomLeft(2));
            glEnd();
            glFlush();

            overlap += CameraPoseGrouperDeviceCode::sumUpBuffer(dArray, rowSums);
            //                overlap -= sum;


            // read back the result
//            cv::Mat img(heightScaled, widthScaled, CV_32FC1);
//            glReadPixels(0,0,widthScaled,heightScaled, GL_RED, GL_FLOAT, img.ptr(0));
            //                            for (int y = 0; y < height; y++)
            //                                for (int x = 0; x < width; x++)
            //                                {
            //                                    if (img.at<float>(y,x) > 0)
            //                                        overlap++;
            //                                }

//            std::cout << refCamId << " " << it->first << std::endl;
//            cv::imshow("rendered plane", img);
//            cv::waitKey(0);
        }

        if (overlap  > 0)
        {
            overlaps.push_back(std::make_pair(it->first, overlap));
        }
//            std::cout << refCamId << ":" << it->first << " overlap = " << overlap << std::endl;
    }
}

}

