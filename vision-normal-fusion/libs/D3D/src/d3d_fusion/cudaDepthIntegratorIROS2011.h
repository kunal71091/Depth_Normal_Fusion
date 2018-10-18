#ifndef DEPTHINTEGRATORIROS2011_H
#define DEPTHINTEGRATORIROS2011_H

#include <Eigen/Dense>

#include <d3d_base/depthMap.h>
#include <d3d_base/grid.h>

namespace D3D_CUDA
{
namespace CudaDepthIntegratorIROS2011DeviceCode
{
    void initTile(int xRes, int yRes, int zRes, float** voxelsD);
    void releaseDeviceMemory(void* addr);
    void allocateAndUploadTransformation(float* transformationH, float** transformationD);
    void allocateAndUploadProjection(float* projectionH, float** projectionD);
    void allocateAndUploadDepthData(float* depthDataH, int rows, int cols, float** depthDataD);
    void integrateDepthMap(float* voxelD, float* boxToGlobalD,
                           int xRes, int yRes, int zRes,
                           float xBegin, float yBegin, float zBegin,
                           float deltaX, float deltaY, float deltaZ,
                           float* depthDataD, int rows, int cols,
                           float maxDepth, float minDepth,
                           float epsilon, float eta, float uncertFact,
                           float* projD);
    void downloadTile(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH);

}
}

namespace D3D
{
class CudaDepthIntegratorIROS2011
{
public:
    CudaDepthIntegratorIROS2011(int xRes, int yRes, int zRes,
                    float xSize, float ySize, float zSize,
                    float xMin, float yMin, float zMin,
                    float minDepth, float maxDepth,
                    float epsilon, float eta, float uncertFact,
                    Eigen::Matrix4f boxToGlobal, int numTiles);

    ~CudaDepthIntegratorIROS2011();

    void setTile(int i);

    void addDepthMap(D3D::DepthMap<float, double>& depthMap);

    D3D::Grid<float> downloadTile();
    float getTileX();
    float getTileY();
    float getTileZ();

private:
    int xRes, yRes, zRes;
    float xSize, ySize, zSize;
    float xMin, yMin, zMin;

    float minDepth, maxDepth;
    float epsilon;
    float eta;
    float uncertFact;

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
