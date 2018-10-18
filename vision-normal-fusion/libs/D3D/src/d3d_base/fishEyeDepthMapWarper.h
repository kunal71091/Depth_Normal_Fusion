#ifndef FISHEYEDEPTHMAPWARPER_H
#define FISHEYEDEPTHMAPWARPER_H

#include <GL/glut.h>

#include <d3d_base/fishEyeDepthMap.h>
#include <d3d_base/fishEyeCameraMatrix.h>
#include <d3d_base/grid.h>

using std::pair;

namespace D3D
{
    template<typename T, typename U>
    class FishEyeDepthMapWarper
    {
    public:
        FishEyeDepthMapWarper(int width, int height, U minDepth, U maxDepth, U xi, U maxDispDiff);
        ~FishEyeDepthMapWarper();

        FishEyeDepthMapWarper(const FishEyeDepthMapWarper& other);
        FishEyeDepthMapWarper& operator= (FishEyeDepthMapWarper& other);

        FishEyeDepthMap<T, U> warpFishEyeDepthMap(const FishEyeDepthMap<T, U>& origFishEyeDepthMap, const FishEyeCameraMatrix<U>& newFishEyeCamera);
//        pair<DepthMap<T,U>, Grid<T> > warpDepthMap(const DepthMap<T, U>& origDepthMap, const Grid<T>& attachedVal, const CameraMatrix<U>& newCamera);
//        pair<DepthMap<T,U>, pair<Grid<T>, Grid<T> > > warpDepthMap(const DepthMap<T, U>& origDepthMap, const Grid<T>& attachedVal1, const Grid<T>& attachedVal2, const CameraMatrix<U>& newCamera);

    private:
        void initializeGL();

        void setUpProjection(const FishEyeCameraMatrix<U>& cam);

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
        U xi;
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

#endif // FISHEYEDEPTHMAPWARPER_H
