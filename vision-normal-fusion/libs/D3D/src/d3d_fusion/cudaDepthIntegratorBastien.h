#ifndef DEPTHINTEGRATOR_H
#define DEPTHINTEGRATOR_H

#include <Eigen/Dense>

#include <d3d_base/depthMap.h>
#include <d3d_base/grid.h>

namespace D3D_CUDA
{
namespace CudaDepthIntegratorBastienDeviceCode
{
    void initTile(int xRes, int yRes, int zRes, float** voxelsD);
    void releaseDeviceMemory(void* addr);
    void allocateAndUploadTransformation(float* transformationH, float** transformationD);
    void allocateAndUploadProjection(float* projectionH, float** projectionD);
    void allocateAndUploadDepthData(float* depthDataH, int rows, int cols, float** depthDataD);
    void allocateAndUploadWeightData(float* weightDataH, int rows, int cols, float** weightDataD);
    void integrateDepthMapWithFixedWeights(float* voxelD, float* boxToGlobalD,
                                           int xRes, int yRes, int zRes,
                                           float xBegin, float yBegin, float zBegin,
                                           float deltaX, float deltaY, float deltaZ,
                                           float* depthDataD, int rows, int cols,
                                           float maxDepth, float minDepth,
                                           float epsilon, float eta, float delta1, float delta2, float delta3, float delta4,
                                           float* projD);
    void integrateDepthMapWithWeights(float* voxelD, float* boxToGlobalD,
                                           int xRes, int yRes, int zRes,
                                           float xBegin, float yBegin, float zBegin,
                                           float deltaX, float deltaY, float deltaZ,
                                           float* depthDataD, int rows, int cols,
                                           float* weightDataD,
                                           float maxDepth, float minDepth,
                                           float epsilon, float eta, float delta1, float delta2, float delta3, float delta4,
                                           float* projD);
    void downloadTile(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH);

}
}

namespace D3D
{
class CudaDepthIntegratorBastien
{
public:
    CudaDepthIntegratorBastien(int xRes, int yRes, int zRes,
                    float xSize, float ySize, float zSize,
                    float xMin, float yMin, float zMin,
                    float minDepth, float maxDepth,
                    Eigen::Matrix4f boxToGlobal, int numTiles);

    ~CudaDepthIntegratorBastien();

    void setTile(int i);

    void setFixedWeights(float epsilon, float eta, float delta1, float delta2, float delta3, float delta4);

    void addDepthMapWithFixedWeights(D3D::DepthMap<float, double>& depthMap);

    void addDepthMapWithWeights(D3D::DepthMap<float, double>& depthMap,D3D::DepthMap<float, double>& weights);

    D3D::Grid<float> downloadTile();
    float getTileX();
    float getTileY();
    float getTileZ();

private:
    int xRes, yRes, zRes;
    float xSize, ySize, zSize;
    float xMin, yMin, zMin;

    float minDepth, maxDepth;

                          //  epsilon-|                                               _________
                          //          |                                              /         \
                          //          |                                             /           \
                          //          |                                            /             \
                          //          |                                           /               \
                          //          |                                          /                 \
                          //        0-|                         ________________/                   \___
                          //          |                        /
                          //          |                       /        !
                          //-eps * eta|__________            /
    bool fixedWeightsSet; //          |          |          /          !
    float epsilon;        //          |          |         /
    float eta;            //          |          |________/            !
    float delta1;         // -epsilon-|==//==============================================================>
    float delta2;         //                     |       |     |       |       |      |        |     |
    float delta3;         //                 -delta3 -delta2 -delta1 depth  delta1  delta2  delta3  delta4
    float delta4;

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
