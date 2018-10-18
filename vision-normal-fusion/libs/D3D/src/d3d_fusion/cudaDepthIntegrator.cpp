#include "cudaDepthIntegrator.h"
#include <d3d_base/exception.h>
#include <d3d_base/grid.h>

using namespace D3D_CUDA::CudaDepthIntegratorDeviceCode;
using namespace D3D;

CudaDepthIntegrator::CudaDepthIntegrator(int xRes, int yRes, int zRes,
                                 float xSize, float ySize, float zSize,
                                 float xMin, float yMin, float zMin,
                                 float minDepth, float maxDepth,
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
    this->boxToGlobal = boxToGlobal;
    this->numTiles = numTiles;
    this->fixedWeightsSet = false;

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
#ifdef DEBUG
    allocateAndUploadTransformationDebug(boxToGlobalA, &boxToGlobalD);
#else
    allocateAndUploadTransformation(boxToGlobalA, &boxToGlobalD);
#endif

}

CudaDepthIntegrator::~CudaDepthIntegrator()
{
#ifdef DEBUG
    if (currentTile >= 0)
        releaseDeviceMemoryDebug(voxelsTD);

    releaseDeviceMemoryDebug(boxToGlobalD);
#else
    if (currentTile >= 0)
        releaseDeviceMemory(voxelsTD);

    releaseDeviceMemory(boxToGlobalD);
#endif

}

void CudaDepthIntegrator::setFixedWeights(float epsilon, float eta, float delta1, float delta2, float delta3)
{
    this->epsilon = epsilon;
    this->eta = eta;
    this->delta1 = delta1;
    this->delta2 = delta2;
    this->delta3 = delta3;
    fixedWeightsSet = true;
}


void CudaDepthIntegrator::setTile(int i)
{
#ifdef DEBUG
    if (currentTile >= 0)
        releaseDeviceMemoryDebug(voxelsTD);
#else
    if (currentTile >= 0)
        releaseDeviceMemory(voxelsTD);
#endif

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

#ifdef DEBUG
    initTileDebug(xResT, yResT, zResT, &voxelsTD);
#else
    initTile(xResT, yResT, zResT, &voxelsTD);
#endif

    currentTile = i;
}

void CudaDepthIntegrator::addDepthMapWithFixedWeights(D3D::DepthMap<float, double> &depthMap)
{
    if (!fixedWeightsSet)
    {
        D3D_THROW_EXCEPTION("Fixed weights need to be set before using this function.")
    }

    if (currentTile < 0)
    {
        D3D_THROW_EXCEPTION("A tile needs to be set before adding any depth maps.")
    }

    // upload projection
    float* projectionD;
    float projectionA[12];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            projectionA[i*4 + j] = depthMap.getCam().getP()(i,j);
#ifdef DEBUG
    allocateAndUploadProjectionDebug(projectionA, &projectionD);
#else
    allocateAndUploadProjection(projectionA, &projectionD);
#endif


    // upload depth data
    float* depthDataD;
#ifdef DEBUG
    allocateAndUploadDepthDataDebug(depthMap.getDataPtr(), depthMap.getHeight(), depthMap.getWidth(), &depthDataD);
#else
    allocateAndUploadDepthData(depthMap.getDataPtr(), depthMap.getHeight(), depthMap.getWidth(), &depthDataD);
#endif

    integrateDepthMapWithFixedWeights(voxelsTD, boxToGlobalD,
                                      xResT, yResT, zResT,
                                      xBeginT, yBeginT, zBeginT,
                                      deltaX, deltaY, deltaZ,
                                      depthDataD, depthMap.getHeight(), depthMap.getWidth(),
                                      maxDepth, minDepth,
                                      epsilon, eta, delta1, delta2, delta3,
                                      projectionD);

#ifdef DEBUG
    releaseDeviceMemoryDebug(projectionD);
    releaseDeviceMemoryDebug(depthDataD);
#else
    // relase memory
    releaseDeviceMemory(projectionD);
    releaseDeviceMemory(depthDataD);
#endif


}

float CudaDepthIntegrator::getTileX()
{
    return tileX;
}

float CudaDepthIntegrator::getTileY()
{
    return tileY;
}

float CudaDepthIntegrator::getTileZ()
{
    return tileZ;
}

D3D::Grid<float> CudaDepthIntegrator::downloadTile()
{
    D3D::Grid<float> voxels(xResT, yResT, zResT);
#ifdef DEBUG
    D3D_CUDA::CudaDepthIntegratorDeviceCode::downloadTileDebug(xResT, yResT, zResT, voxelsTD, voxels.getDataPtr());
#else
     D3D_CUDA::CudaDepthIntegratorDeviceCode::downloadTile(xResT, yResT, zResT, voxelsTD, voxels.getDataPtr());
#endif

    return voxels;
}

