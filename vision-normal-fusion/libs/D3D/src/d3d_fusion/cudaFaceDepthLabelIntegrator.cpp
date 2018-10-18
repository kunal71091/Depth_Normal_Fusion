#include "cudaFaceDepthLabelIntegrator.h"
#include <d3d_base/exception.h>
#include <d3d_base/grid.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace D3D_CUDA::CudaFaceDepthLabelIntegratorDeviceCode;
using namespace D3D;

CudaFaceDepthLabelIntegrator::CudaFaceDepthLabelIntegrator(int numLabels, int xRes, int yRes, int zRes,
                                                           float xSize, float ySize, float zSize,
                                                           float xMin, float yMin, float zMin,
                                                           float minDepth, float maxDepth,
                                                           Eigen::Matrix4f boxToGlobal, int numTiles)
{
    this->numLabels = numLabels;
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

    allocateAndUploadTransformationDebug(boxToGlobalA, &boxToGlobalD);

//    allocateAndUploadTransformation(boxToGlobalA, &boxToGlobalD);

}

CudaFaceDepthLabelIntegrator::~CudaFaceDepthLabelIntegrator()
{
    if (currentTile >= 0) {
        releaseDeviceMemoryDebug(voxelsTD);
        releaseDeviceMemoryDebug(voxelsLTD);
    }

    releaseDeviceMemoryDebug(boxToGlobalD);

//    if (currentTile >= 0)
//        releaseDeviceMemory(voxelsTD);

//    releaseDeviceMemory(boxToGlobalD);
}

void CudaFaceDepthLabelIntegrator::setFixedWeights(float epsilon, float eta, float delta1, float delta2, float delta3)
{
    this->epsilon = epsilon;
    this->eta = eta;
    this->delta1 = delta1;
    this->delta2 = delta2;
    this->delta3 = delta3;
    fixedWeightsSet = true;
}


void CudaFaceDepthLabelIntegrator::setTile(int i)
{
    if (currentTile >= 0) {
        releaseDeviceMemoryDebug(voxelsTD);
        releaseDeviceMemoryDebug(voxelsLTD);
    }

//    if (currentTile >= 0)
//        releaseDeviceMemory(voxelsTD);


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


    initTileDebug(numLabels, xResT, yResT, zResT, &voxelsTD, &voxelsLTD);

//    initTile(xResT, yResT, zResT, &voxelsTD);

    currentTile = i;
}

void CudaFaceDepthLabelIntegrator::addFaceDepthLabels(D3D::DepthMap<float, double> &depthMap, D3D::Grid<double>& unaryResponses /*cv::Mat& labels*/)
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

    allocateAndUploadProjectionDebug(projectionA, &projectionD);

//    allocateAndUploadProjection(projectionA, &projectionD);

    // upload depth data
    float* depthDataD;
    allocateAndUploadDepthDataDebug(depthMap.getDataPtr(), depthMap.getHeight(), depthMap.getWidth(), &depthDataD);

//    int* labelsD = new int[labels.cols*labels.rows];
//    for (int y = 0; y < labels.rows; ++y) {
//        cv::Vec3b *row_ptr = labels.ptr<cv::Vec3b>(y);
//        for (int x = 0; x < labels.cols; ++x) {
//            if (row_ptr[x][2] == 255 && row_ptr[x][1] == 255 && row_ptr[x][0] == 255) {
//                // Skin
//                labelsD[y*labels.cols + x] = 1;
//            } else if (row_ptr[x][2] == 0 && row_ptr[x][1] == 0 && row_ptr[x][0] == 0) {
//                // Hair
//                labelsD[y*labels.cols + x] = 2;
//            } else if (row_ptr[x][2] == 255 && row_ptr[x][1] == 0 && row_ptr[x][0] == 0) {
//                // Beard
//                labelsD[y*labels.cols + x] = 3;
//            } else if (row_ptr[x][2] == 0 && row_ptr[x][1] == 0 && row_ptr[x][0] == 255) {
//                // Eyebrow
//                labelsD[y*labels.cols + x] = 4;
//            } else if (row_ptr[x][2] == 0 && row_ptr[x][1] == 255 && row_ptr[x][0] == 0) {
//                // Eyeball
//                labelsD[y*labels.cols + x] = 5;
//            } else {
//                // Background
//                labelsD[y*labels.cols + x] = 0;
//            }
//        }
//    }

    const int width = unaryResponses.getWidth();
    const int height = unaryResponses.getHeight();

//    if (numLabels != unaryResponses.getDepth()) {
//        D3D_THROW_EXCEPTION("Number of labels in unaryResponses does not match expected number of labels");
//    }

//    cv::Mat labelDebug(height, width, CV_8UC3);

    float* labelsD = new float[width*height*numLabels];
    float* labelsMinD = new float[width*height];
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            float minVal = std::numeric_limits<float>::max();
            int minLabel = -1;
            for (int l = 0; l < numLabels; ++l) {
                double val = unaryResponses(x, y, l);
                labelsD[l*width*height + y*width + x] = val;
                if (val < minVal) {
                    minVal = val;
                    minLabel = l;
                }
            }
            labelsMinD[y*width+x] = minLabel;

//            unsigned char rgb[3];
//            switch(minLabel)
//            {
//                case 0:
//                    rgb[0] = 255; rgb[1] = 255; rgb[2] = 255; break;
//                case 1:
//                    rgb[0] = 0; rgb[1] = 0; rgb[2] = 0; break;
//                case 2:
//                    rgb[0] = 255; rgb[1] = 0; rgb[2] = 0; break;
//                case 3:
//                    rgb[0] = 0; rgb[1] = 0; rgb[2] = 255; break;
//                case 4:
//                    rgb[0] = 0; rgb[1] = 255; rgb[2] = 0; break;
//                case 5:
//                    rgb[0] = 190; rgb[1] = 190; rgb[2] = 190; break;
//                default:
//                    rgb[0] = 50; rgb[1] = 50; rgb[2] = 50; break;
//            }
//            labelDebug.at<cv::Vec3b>(y, x) = cv::Vec3b(rgb[2], rgb[1], rgb[0]);
        }
    }
//    cv::imshow("", labelDebug);
//    cv::waitKey(1000);



//    allocateAndUploadDepthData(depthMap.getDataPtr(), depthMap.getHeight(), depthMap.getWidth(), &depthDataD);

    integrateFaceDepthLabelsDebug(voxelsTD, voxelsLTD, boxToGlobalD,
                                  numLabels, xResT, yResT, zResT,
                                  xBeginT, yBeginT, zBeginT,
                                  deltaX, deltaY, deltaZ,
                                  depthDataD, labelsD, labelsMinD, height, width,
                                  maxDepth, minDepth,
                                  epsilon, eta, delta1, delta2, delta3,
                                  projectionD);

    releaseDeviceMemoryDebug(projectionD);
    releaseDeviceMemoryDebug(depthDataD);

    delete [] labelsD;
    delete [] labelsMinD;

//    // relase memory
//    releaseDeviceMemory(projectionD);
//    releaseDeviceMemory(depthDataD);
}

float CudaFaceDepthLabelIntegrator::getTileX()
{
    return tileX;
}

float CudaFaceDepthLabelIntegrator::getTileY()
{
    return tileY;
}

float CudaFaceDepthLabelIntegrator::getTileZ()
{
    return tileZ;
}

std::pair<D3D::Grid<float>, D3D::Grid<float> > CudaFaceDepthLabelIntegrator::downloadTile()
{
    D3D::Grid<float> voxels(xResT, yResT, zResT);
    D3D::Grid<float> voxelsL(numLabels*xResT, yResT, zResT);
    D3D_CUDA::CudaFaceDepthLabelIntegratorDeviceCode::downloadTileDebug(numLabels, xResT, yResT, zResT,
                                                                        voxelsTD, voxels.getDataPtr(),
                                                                        voxelsLTD, voxelsL.getDataPtr());

//    D3D_CUDA::CudaLabelIntegratorDeviceCode::downloadTile(xResT, yResT, zResT, voxelsTD, voxels.getDataPtr());

    return std::make_pair(voxels, voxelsL);
}

