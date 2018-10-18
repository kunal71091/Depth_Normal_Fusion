#include "cudaDepthIntegratorIROS2011.h"
#include <d3d_base/exception.h>
#include <d3d_base/grid.h>


using namespace D3D_CUDA::CudaDepthIntegratorIROS2011DeviceCode;
using namespace D3D;

CudaDepthIntegratorIROS2011::CudaDepthIntegratorIROS2011(int xRes, int yRes, int zRes,
                                 float xSize, float ySize, float zSize,
                                 float xMin, float yMin, float zMin,
                                 float minDepth, float maxDepth,
                                 float epsilon, float eta, float uncertFact,
                                 Eigen::Matrix4f boxToGlobal, int numTiles)
{
    this->xRes = xRes;
    this->yRes = yRes;
    this->zRes = zRes;

    if (zRes < 16)
    {
        D3D_THROW_EXCEPTION("Minimal resolution in z-direction is 16")
    }

    this->xSize = xSize;
    this->ySize = ySize;
    this->zSize = zSize;
    this->xMin = xMin;
    this->yMin = yMin;
    this->zMin = zMin;
    this->minDepth = minDepth;
    this->maxDepth = maxDepth;
    this->epsilon = epsilon;
    this->eta = eta;
    this->uncertFact = uncertFact;
    this->boxToGlobal = boxToGlobal;
    this->numTiles = numTiles;

    this->deltaX = xSize/xRes;
    this->deltaY = ySize/yRes;
    this->deltaZ = zSize/zRes;

    currentTile = -1; // no tile active

    tileXRes = xRes % numTiles == 0 ? xRes/numTiles : xRes/numTiles + 1;

    // upload box to global transformation to device
    float boxToGlobalA[16];
    for (int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            boxToGlobalA[i*4+j] = boxToGlobal(i,j);
    allocateAndUploadTransformation(boxToGlobalA, &boxToGlobalD);
}

CudaDepthIntegratorIROS2011::~CudaDepthIntegratorIROS2011()
{
    if (currentTile >= 0)
        releaseDeviceMemory(voxelsTD);

    releaseDeviceMemory(boxToGlobalD);
}


void CudaDepthIntegratorIROS2011::setTile(int i)
{
    if (currentTile >= 0)
        releaseDeviceMemory(voxelsTD);

    // compute start
    xBeginT = xMin + i*tileXRes*deltaX + deltaX / 2.0f;
    yBeginT = yMin + deltaY / 2.0f;
    zBeginT = zMin + deltaZ / 2.0f;

    tileX = i*tileXRes;
    tileY = 0;
    tileZ = 0;

    xResT = std::min(tileXRes, xRes - i*tileXRes);
    yResT = yRes;
    zResT = zRes;

    initTile(xResT, yResT, zResT, &voxelsTD);
    currentTile = i;
}

void CudaDepthIntegratorIROS2011::addDepthMap(D3D::DepthMap<float, double> &depthMap)
{
    // upload projection
    float* projectionD;
    float projectionA[12];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            projectionA[i*4 + j] = depthMap.getCam().getP()(i,j);
    allocateAndUploadProjection(projectionA, &projectionD);

    // upload depth data
    float* depthDataD;
    allocateAndUploadDepthData(depthMap.getDataPtr(), depthMap.getHeight(), depthMap.getWidth(), &depthDataD);

    integrateDepthMap(voxelsTD, boxToGlobalD,
                      xResT, yResT, zResT,
                      xBeginT, yBeginT, zBeginT,
                      deltaX, deltaY, deltaZ,
                      depthDataD, depthMap.getHeight(), depthMap.getWidth(),
                      maxDepth, minDepth,
                      epsilon, eta, uncertFact,
                      projectionD);



    // relase memory
    releaseDeviceMemory(projectionD);
    releaseDeviceMemory(depthDataD);
}

float CudaDepthIntegratorIROS2011::getTileX()
{
    return tileX;
}

float CudaDepthIntegratorIROS2011::getTileY()
{
    return tileY;
}

float CudaDepthIntegratorIROS2011::getTileZ()
{
    return tileZ;
}

D3D::Grid<float> CudaDepthIntegratorIROS2011::downloadTile()
{
    D3D::Grid<float> voxels(xResT, yResT, zResT);
    D3D_CUDA::CudaDepthIntegratorIROS2011DeviceCode::downloadTile(xResT, yResT, zResT, voxelsTD, voxels.getDataPtr());
    return voxels;
}

