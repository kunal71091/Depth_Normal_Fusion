#ifndef LABELINTEGRATOR_H
#define LABELINTEGRATOR_H

#include <Eigen/Dense>

#include <d3d_base/depthMap.h>
#include <d3d_base/grid.h>

namespace D3D_CUDA
{
namespace CudaFaceDepthLabelIntegratorDeviceCode
{
//    void initTile(int xRes, int yRes, int zRes, float** voxelsD);
//    void releaseDeviceMemory(void* addr);
//    void allocateAndUploadTransformation(float* transformationH, float** transformationD);
//    void allocateAndUploadProjection(float* projectionH, float** projectionD);
//    void allocateAndUploadDepthData(float* depthDataH, int rows, int cols, float** depthDataD);
//    void integrateLabels(float* voxelD, float* boxToGlobalD,
//                                           int xRes, int yRes, int zRes,
//                                           float xBegin, float yBegin, float zBegin,
//                                           float deltaX, float deltaY, float deltaZ,
//                                           float* depthDataD, int rows, int cols,
//                                           float maxDepth, float minDepth,
//                                           float epsilon, float eta, float delta1, float delta2, float delta3,
//                                           float* projD);
//    void downloadTile(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH);


    void initTileDebug(int numLabels, int xRes, int yRes, int zRes, float** voxelsD, float** voxelsLD);
    void releaseDeviceMemoryDebug(void* addr);
    void allocateAndUploadTransformationDebug(float* transformationH, float** transformationD);
    void allocateAndUploadProjectionDebug(float* projectionH, float** projectionD);
    void allocateAndUploadDepthDataDebug(float* depthDataH, int rows, int cols, float** depthDataD);
    void integrateFaceDepthLabelsDebug(float* voxelD, float* voxelLD, float* boxToGlobalD,
                                       int numLabels, int xRes, int yRes, int zRes,
                                       float xBegin, float yBegin, float zBegin,
                                       float deltaX, float deltaY, float deltaZ,
                                       float* depthDataD, float* labelsD, float* labelsMinD, int rows, int cols,
                                       float maxDepth, float minDepth,
                                       float epsilon, float eta, float delta1, float delta2, float delta3,
                                       float* projD);
    void downloadTileDebug(int numLabels, int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH, float* voxelsLD, float* voxelsLH);

}
}

namespace D3D
{
class CudaFaceDepthLabelIntegrator
{
public:
    CudaFaceDepthLabelIntegrator(int numLabels, int xRes, int yRes, int zRes,
                        float xSize, float ySize, float zSize,
                        float xMin, float yMin, float zMin,
                        float minDepth, float maxDepth,
                        Eigen::Matrix4f boxToGlobal, int numTiles);

    ~CudaFaceDepthLabelIntegrator();

    void setTile(int i);

    void setFixedWeights(float epsilon, float eta, float delta1, float delta2, float delta3);

    void addFaceDepthLabels(D3D::DepthMap<float, double>& depthMap, D3D::Grid<double> &unaryResponses);

    std::pair<D3D::Grid<float>, D3D::Grid<float> > downloadTile();
    float getTileX();
    float getTileY();
    float getTileZ();

private:
    int numLabels;
    int xRes, yRes, zRes;
    float xSize, ySize, zSize;
    float xMin, yMin, zMin;

    float minDepth, maxDepth;

                          //  epsilon-|                                               _________
                          //          |                                              /         |
                          //          |                                             /          |
                          //          |                                            /           |
                          //          |                                           /            |
                          //          |                                          /             |
                          //        0-|                         ________________/              |_____
                          //          |                        /
                          //          |                       /        |
                          //-eps * eta|__________            /
    bool fixedWeightsSet; //          |          |          /          |
    float epsilon;        //          |          |         /
    float eta;            //          |          |________/            |
    float delta1;         // -epsilon-|==//=======================================================>
    float delta2;         //                     |       |     |       |       |      |        |
    float delta3;         //                 -delta3 -delta2 -delta1 depth  delta1  delta2  delta3

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
    float* voxelsLTD;
};

}

#endif
