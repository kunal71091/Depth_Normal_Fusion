#include "cudaDepthIntegratorBastien.h"
#include <d3d_base/exception.h>
#include <d3d_base/grid.h>


using namespace D3D_CUDA::CudaDepthIntegratorBastienDeviceCode;
using namespace D3D;

CudaDepthIntegratorBastien::CudaDepthIntegratorBastien(int xRes, int yRes, int zRes,
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
    allocateAndUploadTransformation(boxToGlobalA, &boxToGlobalD);
}

CudaDepthIntegratorBastien::~CudaDepthIntegratorBastien()
{
    if (currentTile >= 0)
        releaseDeviceMemory(voxelsTD);

    releaseDeviceMemory(boxToGlobalD);
}

void CudaDepthIntegratorBastien::setFixedWeights(float epsilon, float eta, float delta1, float delta2, float delta3, float delta4)
{
    this->epsilon = epsilon;
    this->eta = eta;
    this->delta1 = delta1;
    this->delta2 = delta2;
    this->delta3 = delta3;
    this->delta4 = delta4;
    fixedWeightsSet = true;
}


void CudaDepthIntegratorBastien::setTile(int i)
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

void CudaDepthIntegratorBastien::addDepthMapWithFixedWeights(D3D::DepthMap<float, double> &depthMap)
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
    allocateAndUploadProjection(projectionA, &projectionD);

    // upload depth data
    float* depthDataD;
    allocateAndUploadDepthData(depthMap.getDataPtr(), depthMap.getHeight(), depthMap.getWidth(), &depthDataD);

    integrateDepthMapWithFixedWeights(voxelsTD, boxToGlobalD,
                                      xResT, yResT, zResT,
                                      xBeginT, yBeginT, zBeginT,
                                      deltaX, deltaY, deltaZ,
                                      depthDataD, depthMap.getHeight(), depthMap.getWidth(),
                                      maxDepth, minDepth,
                                      epsilon, eta, delta1, delta2, delta3, delta4,
                                      projectionD);



    // relase memory
    releaseDeviceMemory(projectionD);
    releaseDeviceMemory(depthDataD);
}

void CudaDepthIntegratorBastien::addDepthMapWithWeights(D3D::DepthMap<float, double> &depthMap,D3D::DepthMap<float, double> &weights)
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
    allocateAndUploadProjection(projectionA, &projectionD);

    // upload depth data
    float* depthDataD;
    allocateAndUploadDepthData(depthMap.getDataPtr(), depthMap.getHeight(), depthMap.getWidth(), &depthDataD);

    float* weightsDataD;
    allocateAndUploadWeightData(weights.getDataPtr(), weights.getHeight(), weights.getWidth(), &weightsDataD);

    integrateDepthMapWithWeights(voxelsTD, boxToGlobalD,
                                      xResT, yResT, zResT,
                                      xBeginT, yBeginT, zBeginT,
                                      deltaX, deltaY, deltaZ,
                                      depthDataD, depthMap.getHeight(), depthMap.getWidth(),
                                      weightsDataD,
                                      maxDepth, minDepth,
                                      epsilon, eta, delta1, delta2, delta3, delta4,
                                      projectionD);



    // relase memory
    releaseDeviceMemory(projectionD);
    releaseDeviceMemory(depthDataD);
    releaseDeviceMemory(weightsDataD);
}

float CudaDepthIntegratorBastien::getTileX()
{
    return tileX;
}

float CudaDepthIntegratorBastien::getTileY()
{
    return tileY;
}

float CudaDepthIntegratorBastien::getTileZ()
{
    return tileZ;
}

D3D::Grid<float> CudaDepthIntegratorBastien::downloadTile()
{
    D3D::Grid<float> voxels(xResT, yResT, zResT);
    D3D_CUDA::CudaDepthIntegratorBastienDeviceCode::downloadTile(xResT, yResT, zResT, voxelsTD, voxels.getDataPtr());
    return voxels;
}

