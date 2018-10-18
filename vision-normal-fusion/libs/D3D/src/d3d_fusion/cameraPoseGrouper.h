#ifndef CAMERAPOSEGROUPER_H
#define CAMERAPOSEGROUPER_H

#include <d3d_base/cameraMatrix.h>
#include <map>
#include <vector>
#include <GL/glew.h>
#include <GL/glut.h>
#include <cuda_gl_interop.h>

namespace D3D_CUDA
{
namespace CameraPoseGrouperDeviceCode
{
    void setUpCudaInterop(GLuint bufferId, cudaGraphicsResource** bufRes, cudaArray** dArray, float** tempGlobMem);
    void cleanUpCudaInterop(struct cudaGraphicsResource** bufRes, float** tempGlobMem);
    float sumUpBuffer(struct cudaArray* dArray, float* rowSums);
}
}

namespace D3D
{

class CameraPoseGrouper
{
public:
    CameraPoseGrouper();
    ~CameraPoseGrouper();

    void addCamera(int id, const D3D::CameraMatrix<double>& cam);
    void setViewingRange(float minZ, float maxZ);
    void setNumPlanes(int numPlanes);
    void setScale(float scale);
    void setSize(int width, int height);

    void computeOverlaps(int refCamId, std::vector<std::pair<int, float> > &overlaps);

private:
    void initializeGLCuda();
    void cleanUpGLCuda();

    std::map<int, D3D::CameraMatrix<double> > cameras;
    int numPlanes;
    float minZ;
    float maxZ;
    float scale;
    int width;
    int height;
    bool initialized;

    int widthScaled;
    int heightScaled;

    //gl variables
    GLuint fbo;
    GLuint depthbuffer;
    GLuint colorbuffer;
    GLuint fragShader;
    GLuint vertShader;
    GLuint shaderProgram;

    // cuda interop
    struct cudaGraphicsResource* bufRes;
    struct cudaArray* dArray;

    // temp buffer for rowSums
    float* rowSums;

};
}

#endif // CAMERAPOSEGROUPER_H
