#ifndef DEPTHINTEGRATORMULTICLASS_H
#define DEPTHINTEGRATORMULTICLASS_H

#include <Eigen/Dense>

#include <d3d_base/depthMap.h>
#include <d3d_base/grid.h>

namespace D3D_CUDA
{
namespace CudaDepthIntegratorMultiClassv2DeviceCode
{
    void initTile(int xRes, int yRes, int zRes, int numClasses, float** voxelsD);
    void releaseDeviceMemory(void* addr);
    void allocateAndUploadTransformation(float* transformationH, float** transformationD);
    void allocateAndUploadProjection(float* projectionH, float** projectionD);
    void allocateAndUploadClassScoresData(float* classScoresDataH, int rows, int cols, int numClasses, float** classScoresDataD);
    void allocateAndUploadDepthData(float* depthDataH, int rows, int cols, float** depthDataD);
    void integrateDepthMap(float* voxelD, const float* boxToGlobalD,
                           int xRes, int yRes, int zRes,
                           int numClasses, int freeSpaceClass,
                           float xBegin, float yBegin, float zBegin,
                           float deltaX, float deltaY, float deltaZ,
                           const float* classScoresDataD, const float* depthDataD, int rows, int cols,
                           float maxDepth, float minDepth,
                           float epsilon, float eta, float uncertFact, float rho,
                           const float* projD);
    void downloadTile(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH);

}
}

namespace D3D
{
class CudaDepthIntegratorMultiClassv2
{
public:
    CudaDepthIntegratorMultiClassv2(int xRes, int yRes, int zRes,
                    float xSize, float ySize, float zSize,
                    int numClasses, int freeSpaceClass,
                    float xMin, float yMin, float zMin,
                    float minDepth, float maxDepth,
                    float epsilon, float eta, float uncertFact,
                    float rho, Eigen::Matrix4f boxToGlobal, int numTiles);

    ~CudaDepthIntegratorMultiClassv2();

    void setTile(int i);

    void addDepthMap(D3D::DepthMap<float, double>& depthMap, D3D::Grid<float>& classScores);

    D3D::Grid<float> downloadTile();
    float getTileX();
    float getTileY();
    float getTileZ();

private:
    int _xRes, _yRes, _zRes;
    float _xSize, _ySize, _zSize;
    float _xMin, _yMin, _zMin;

    int _numClasses;
    int _freeSpaceClass;

    float _minDepth, _maxDepth;
    float _epsilon;
    float _eta;
    float _uncertFact;
    float _rho;

    Eigen::Matrix4f _boxToGlobal;
    float* _boxToGlobalD;

    int _numTiles; // tiling of the volume in case it does not fit in gpu memory at once

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
