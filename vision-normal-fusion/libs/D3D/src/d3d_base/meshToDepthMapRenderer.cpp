#include <GL/glew.h>

#include "meshToDepthMapRenderer.h"
#include <d3d_glBase/glCommon.h>

namespace D3D
{

template <typename T, typename U>
MeshToDepthMapRenderer<T, U>::MeshToDepthMapRenderer(int width, int height, U minDepth, U maxDepth)
{
    this->width = width;
    this->height = height;
    this->minDepth = minDepth;
    this->maxDepth = maxDepth;
    initializeGL();
    meshDisplayList = 0;
}

template <typename T, typename U>
MeshToDepthMapRenderer<T, U>::~MeshToDepthMapRenderer()
{
    if (meshDisplayList > 0)
    {
        glDeleteLists(meshDisplayList, 1);
        D3D_GL_CHECK_ERROR;
    }
}

template <typename T, typename U>
void MeshToDepthMapRenderer<T, U>::initializeGL()
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
void MeshToDepthMapRenderer<T, U>::setUpProjection(const CameraMatrix<U> &cam)
{
    // set projection
    glMatrixMode(GL_PROJECTION);

    // assumes that K(2,2) is 1
    const float fx = cam.getK()(0,0);
    const float fy = cam.getK()(1,1);
    const float px = cam.getK()(0,2);
    const float py = cam.getK()(1,2);

    float m[16];


      // Version from Thomas Schoeps
    m[0] = (2*fx)/width;   m[4] = 0;               m[8] = 2*(0.5f + px)/width - 1.0f;                 m[12] = 0;
    m[1] = 0;              m[5] = (2*fy)/height;   m[9] = 2*(0.5f + py)/height - 1.0f;                m[13] = 0;
    m[2] = 0;              m[6] = 0;               m[10] = (maxDepth+minDepth)/(maxDepth-minDepth);   m[14] = -(2*maxDepth*minDepth)/(maxDepth-minDepth);
    m[3] = 0;              m[7] = 0;               m[11] = 1;                                         m[15] = 0;


//    m[0] = 2*fx*(width-2)/(width*(width-1)); m[4] = 0;                                         m[8] = 2*px*(width-2)/(width*(width-1)) - 1 + 1/width;       m[12] = 0;
//    m[1] = 0;                                m[5] = 2*fy*(height-2)/(height*(height-1));       m[9] = 2*py*(height-2)/(height*(height-1)) - 1 + 1/height;   m[13] = 0;
//    m[2] = 0;                                m[6] = 0;                                         m[10] = (maxDepth+minDepth)/(maxDepth-minDepth);             m[14] = -(2*maxDepth*minDepth)/(maxDepth-minDepth);
//    m[3] = 0;                                m[7] = 0;                                         m[11] = 1;                                                   m[15] = 0;

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

template <typename T, typename U>
void MeshToDepthMapRenderer<T, U>::uploadMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::vector<int> >& faces)
{
    if (meshDisplayList == 0)
    {
        meshDisplayList = glGenLists(1);
        D3D_GL_CHECK_ERROR
    }

    glNewList(meshDisplayList, GL_COMPILE);
        glBegin(GL_TRIANGLES);
        for (unsigned int i = 0; i < faces.size(); i++)
        {
            if (faces[i].size() < 3)
            {
                // degenerate triangles are ignored
            }
            if (faces[i].size() == 3)
            {
                // render triangle
                for (int j = 0; j < 3; j++)
                {
                    glVertex3f(vertices[faces[i][j]](0), vertices[faces[i][j]](1), vertices[faces[i][j]](2));
                }
            }
            else
            {
                // render polygon in triangulated form
                for (unsigned int j = 2; j < faces[i].size(); j++)
                {
                    glVertex3d(vertices[faces[i][0]](0), vertices[faces[i][0]](1), vertices[faces[i][0]](2));
                    glVertex3d(vertices[faces[i][j-1]](0), vertices[faces[i][j-1]](1), vertices[faces[i][j-1]](2));
                    glVertex3d(vertices[faces[i][j]](0), vertices[faces[i][j]](1), vertices[faces[i][j]](2));
                }
            }
        }
        glEnd();
    glEndList();
    D3D_GL_CHECK_ERROR;
}

template <typename T, typename U>
D3D::DepthMap<T, U> MeshToDepthMapRenderer<T, U>::renderFromViewpoint(const CameraMatrix<U> &cam)
{
    setUpProjection(cam);

    // set and clear the buffers
    glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);

    if (meshDisplayList == 0)
    {
        D3D_THROW_EXCEPTION("Before rendering mesh needs to be uploaded.")
    }

    glCallList(meshDisplayList);
    glFlush();
    D3D_GL_CHECK_ERROR

    DepthMap<T, U> renderedDepthMap(width, height, cam);
    readResultBack(renderedDepthMap.getDataPtr());
    D3D_GL_CHECK_ERROR

    return renderedDepthMap;
}

template <typename T, typename U>
void MeshToDepthMapRenderer<T, U>::readResultBack(float* dataPtr)
{
    glReadPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_FLOAT, dataPtr);

    for (float* ptr = dataPtr; ptr < dataPtr + width*height; ++ptr)
        *ptr = (*ptr) * maxDepth;

}

template <typename T, typename U>
void MeshToDepthMapRenderer<T, U>::readResultBack(double* dataPtr)
{
    glReadPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_DOUBLE, dataPtr);

    for (double* ptr = dataPtr; ptr < dataPtr + width*height; ++ptr)
        *ptr = (*ptr) * maxDepth;
}

template class MeshToDepthMapRenderer<float, float>;
template class MeshToDepthMapRenderer<float, double>;
template class MeshToDepthMapRenderer<double, float>;
template class MeshToDepthMapRenderer<double, double>;

}
