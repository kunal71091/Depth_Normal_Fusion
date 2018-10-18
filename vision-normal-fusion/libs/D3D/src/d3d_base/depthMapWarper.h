#ifndef DEPTHMAPWARPER_H
#define DEPTHMAPWARPER_H

#include <GL/glut.h>

#include <d3d_base/depthMap.h>
#include <d3d_base/cameraMatrix.h>
#include <d3d_base/grid.h>

using std::pair;

namespace D3D
{
    template<typename T, typename U>
    class DepthMapWarper
    {
    public:
        DepthMapWarper(int width, int height, U minDepth, U maxDepth, U maxDispDiff);
        ~DepthMapWarper();

        DepthMapWarper(const DepthMapWarper& other);
        DepthMapWarper& operator= (DepthMapWarper& other);

        DepthMap<T, U> warpDepthMap(const DepthMap<T, U>& origDepthMap, const CameraMatrix<U>& newCamera);
        pair<DepthMap<T,U>, Grid<T> > warpDepthMap(const DepthMap<T, U>& origDepthMap, const Grid<T>& attachedVal, const CameraMatrix<U>& newCamera);
        pair<DepthMap<T,U>, pair<Grid<T>, Grid<T> > > warpDepthMap(const DepthMap<T, U>& origDepthMap, const Grid<T>& attachedVal1, const Grid<T>& attachedVal2, const CameraMatrix<U>& newCamera);

    private:
        void initializeGL();

        void setUpProjection(const CameraMatrix<U>& cam);

        void readResultBack(float* dataPtr);
        void readResultBack(double* dataPtr);

        void readAttachedVal1Back(float* dataPtr);
        void readAttachedVal1Back(double* dataPtr);

        void readAttachedVal2Back(float* dataPtr);
        void readAttachedVal2Back(double* dataPtr);

        int width;
        int height;

        U minDepth;
        U maxDepth;
        U maxDispDiff;

        GLuint fbo;
        GLuint depthbuffer;
        GLuint colorbuffer;
        GLuint fragShader;
        GLuint vertShader;
        GLuint shaderProgram;

        unsigned int *count;
    };
}

#endif // DEPTHMAPWARPER_H
