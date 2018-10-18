#include "cudaDepthIntegratorMultiClass.h"
#include <d3d_base/exception.h>
#include <d3d_base/grid.h>
#include <iostream>


using namespace D3D_CUDA::CudaDepthIntegratorMultiClassDeviceCode;
using namespace D3D;

CudaDepthIntegratorMultiClass::CudaDepthIntegratorMultiClass(int xRes, int yRes, int zRes,
                                 float xSize, float ySize, float zSize,
                                 int numClasses, int freeSpaceClass,
                                 float xMin, float yMin, float zMin,
                                 float minDepth, float maxDepth,
                                 float epsilon, float eta, float uncertFact, float rho,
                                 Eigen::Matrix4f boxToGlobal, int numTiles, float beta, float skyWeight)
{
    this->_xRes = xRes;
    this->_yRes = yRes;
    this->_zRes = zRes;

    if (zRes < 16)
    {
        D3D_THROW_EXCEPTION("Minimal resolution in z-direction is 16")
    }

    this->_numClasses = numClasses;
    this->_freeSpaceClass = freeSpaceClass;

    this->_xSize = xSize;
    this->_ySize = ySize;
    this->_zSize = zSize;
    this->_xMin = xMin;
    this->_yMin = yMin;
    this->_zMin = zMin;
    this->_minDepth = minDepth;
    this->_maxDepth = maxDepth;
    this->_epsilon = epsilon;
    this->_eta = eta;
    this->_uncertFact = uncertFact;
    this->_rho = rho;
    this->_beta = beta;
    this->_skyWeight = skyWeight;
    this->_boxToGlobal = boxToGlobal;
    this->_numTiles = numTiles;

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
    allocateAndUploadTransformation(boxToGlobalA, &_boxToGlobalD);
}

CudaDepthIntegratorMultiClass::~CudaDepthIntegratorMultiClass()
{
    if (currentTile >= 0)
        releaseDeviceMemory(voxelsTD);

    releaseDeviceMemory(_boxToGlobalD);
}


void CudaDepthIntegratorMultiClass::setTile(int i)
{
    if (currentTile >= 0)
        releaseDeviceMemory(voxelsTD);

    // compute start
    xBeginT = _xMin + i*tileXRes*deltaX + deltaX / 2.0f;
    yBeginT = _yMin + deltaY / 2.0f;
    zBeginT = _zMin + deltaZ / 2.0f;

    tileX = i*tileXRes;
    tileY = 0;
    tileZ = 0;

    xResT = std::min(tileXRes, _xRes - i*tileXRes);
    yResT = _yRes;
    zResT = _zRes;

    initTile(xResT, yResT, zResT, _numClasses, &voxelsTD);
    currentTile = i;
}

void CudaDepthIntegratorMultiClass::addDepthMap(D3D::DepthMap<float, double> &depthMap, D3D::Grid<float>& classScores)
{
    // check dimensions
    if (depthMap.getHeight() != classScores.getDepth() || depthMap.getWidth() != classScores.getHeight())
    {
        D3D_THROW_EXCEPTION("Depth map and class scores sizes do not match.")
    }
    if ((int) classScores.getWidth() != _numClasses)
    {
        D3D_THROW_EXCEPTION("Numer of classes in class scores does not match the number of classes the depth integrator uses.")
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


    // upload scores data
    float* classScoresDataD;
    allocateAndUploadClassScoresData(classScores.getDataPtr(), classScores.getHeight(), classScores.getWidth(), classScores.getDepth(), &classScoresDataD);

    integrateDepthMap(voxelsTD, _boxToGlobalD,
                      xResT, yResT, zResT,
                      _numClasses, _freeSpaceClass,
                      xBeginT, yBeginT, zBeginT,
                      deltaX, deltaY, deltaZ,
                      classScoresDataD, depthDataD, depthMap.getHeight(), depthMap.getWidth(),
                      _maxDepth, _minDepth,
                      _epsilon, _eta, _uncertFact, _rho,
                      projectionD, depthMap.getCam().getC()(0), depthMap.getCam().getC()(1), depthMap.getCam().getC()(2), _beta, _skyWeight);



    // relase memory
    releaseDeviceMemory(projectionD);
    releaseDeviceMemory(depthDataD);
    releaseDeviceMemory(classScoresDataD);
}

float CudaDepthIntegratorMultiClass::getTileX()
{
    return tileX*_numClasses;
}

float CudaDepthIntegratorMultiClass::getTileY()
{
    return tileY;
}

float CudaDepthIntegratorMultiClass::getTileZ()
{
    return tileZ;
}

D3D::Grid<float> CudaDepthIntegratorMultiClass::downloadTile()
{
    D3D::Grid<float> voxels(xResT*_numClasses, yResT, zResT);
    D3D_CUDA::CudaDepthIntegratorMultiClassDeviceCode::downloadTile(xResT*_numClasses, yResT, zResT, voxelsTD, voxels.getDataPtr());
//    for (unsigned int z = 0; z < voxels.getDepth(); z++)
//        for (unsigned int y = 0; y < voxels.getHeight(); y++)
//            for (unsigned int x = 0; x < voxels.getWidth(); x++)
//            {
//                std::cout << voxels(x,y,z) << " ";
//                if ((x+1) % numClasses == 0)
//                {
//                    std::cout << std::endl;
//                }
//            }
    return voxels;
}

