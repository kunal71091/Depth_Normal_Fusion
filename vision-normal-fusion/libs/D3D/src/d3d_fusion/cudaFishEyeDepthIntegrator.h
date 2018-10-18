#ifndef FISHEYEDEPTHINTEGRATOR_H
#define FISHEYEDEPTHINTEGRATOR_H

#include <Eigen/Dense>

#include <d3d_base/fishEyeDepthMap.h>
#include <d3d_base/grid.h>

namespace D3D_CUDA
{
namespace CudaFishEyeDepthIntegratorDeviceCode
{
    void initTile(int xRes, int yRes, int zRes, float** voxelsD);
    void releaseDeviceMemory(void* addr);
    void allocateAndUploadTransformation(float* transformationH, float** transformationD);
    void allocateAndUploadProjection(float* rMatrixH, float** rMatrixD, float* tVectorH, float** tVectorD, float* kMatrixH, float** kMatrixD);
    void allocateAndUploadDepthData(float* depthDataH, int rows, int cols, float** depthDataD);
    void integrateDepthMapWithFixedWeights(float* voxelD, float* boxToGlobalD,
                                           int xRes, int yRes, int zRes,
                                           float xBegin, float yBegin, float zBegin,
                                           float deltaX, float deltaY, float deltaZ,
                                           float* depthDataD, int rows, int cols,
                                           float maxDepth, float minDepth,
                                           float epsilon, float eta, float delta1, float delta2, float delta3,
                                           float* rMatD, float* tVecD, float* kMatD, float xi);
    void downloadTile(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH);

}
}

namespace D3D
{
class CudaFishEyeDepthIntegrator
{
public:
    CudaFishEyeDepthIntegrator(int xRes, int yRes, int zRes,
                              float xSize, float ySize, float zSize,
                              float xMin, float yMin, float zMin,
                              float minDepth, float maxDepth,
                              Eigen::Matrix4f boxToGlobal, int numTiles);

    ~CudaFishEyeDepthIntegrator();

    void setTile(int i);

    void setFixedWeights(float epsilon, float eta, float delta1, float delta2, float delta3);

    void addDepthMapWithFixedWeights(D3D::FishEyeDepthMap<float, double>& fishEyeDepthMap);

    D3D::Grid<float> downloadTile();
    float getTileX();
    float getTileY();
    float getTileZ();

private:
    int xRes, yRes, zRes;
    float xSize, ySize, zSize;
    float xMin, yMin, zMin;

    float minDepth, maxDepth;

                          //                             _______________        <- epsilon
                          //                            /               |
                          //                           /                |
                          //                          /                 |
                          //                         /                  |
                          //                        /                   |
                          //                       /                    |
                          //                      /                     |
    bool fixedWeightsSet; //                     /                      |
    float epsilon;        //                    /                       |
    float eta;            //                   /                        |
    float delta1;         // ===========================================================>
    float delta2;         //           l       l         l              l
    float delta3;         //           depth   delta1    delta2         delta3

    Eigen::Matrix4f boxToGlobal;
    float* boxToGlobalD;

    int numTiles; // tiling of the volume in case it does not fit in gpu memory at once

    float deltaX;
    float deltaY;
    float deltaZ;

    int tileXRes;

    int currentTile;

    float xBeginT, yBeginT, zBeginT; // first voxel center of current tile
    int tileX, tileY, tileZ; // tile position
    float xResT, yResT, zResT;

    float* voxelsTD; // voxels on the device
};

}

#endif
