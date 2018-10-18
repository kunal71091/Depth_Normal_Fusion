#ifndef CUDAPLANESWEEP_H
#define CUDAPLANESWEEP_H

#include <d3d_base/cameraMatrix.h>
#include <d3d_base/depthMap.h>
#include <d3d_cudaBase/deviceBuffer.h>
#include <d3d_base/grid.h>
#include <d3d_cudaBase/deviceImage.h>

#include <map>
#include <vector>

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>

using std::map;
using std::vector;
using cv::Mat;
using Eigen::Matrix;
using Eigen::Vector4d;


namespace D3D_CUDA
{
    namespace CudaPlaneSweepDeviceCode
    {
        // device function declarations
        void planeSweepInitTexturing();

        void planeSweepBoxFilterCosts(DeviceBuffer<float>& costBuf, DeviceBuffer<float>& filteredCostBuf, int radius_x, int radius_y);
        void planeSweepBoxFilterImageAndSqrImage(DeviceImage& refImg,
                                                 DeviceBuffer<float>& boxFilterBuf, DeviceBuffer<float>& boxFilterSqrBuf,
                                                 int radius_x, int radius_y);
        void planeSweepWarpZNCC(const DeviceImage& otherImg, float* homography, const DeviceImage& refImg,
                                DeviceBuffer<float>& refImgBoxFilterBuf, DeviceBuffer<float>& refImgSqrBoxFilterBuf,
                                DeviceBuffer<float>& costBuf, int radius_x, int radius_y);
        void planeSweepWarpZNCCAccum(const DeviceImage& otherImg, float* homography,
                                    const DeviceImage& refImg,
                                    DeviceBuffer<float>& refImgBoxFilterBuf, DeviceBuffer<float>& refImgSqrBoxFilterBuf,
                                    float accumScale, DeviceBuffer<float>& costAccumBuf, int radius_x, int radius_y);
        void planeSweepWarpAD(const DeviceImage& srcImg, bool color, const float* homography,
                              const DeviceImage& refImg,
                              DeviceBuffer<float>& costBuf);
        void planeSweepWarpADAccum(DeviceImage& srcImg, bool color, const float* homography,
                                   DeviceImage& refImg,
                                   float accumScale, DeviceBuffer<float>& costAccumBuf);
        void updateBestK(DeviceBuffer<float>& newCostsBuf, DeviceBuffer<float>& bestCostsBuf, DeviceBuffer<float>& bestMinBuf);
        void planeSweepAccumCostBestK(DeviceBuffer<float>& costAccumBuf, DeviceBuffer<float>& costBuf, DeviceBuffer<float>& minCostBuf, float maxVal, float accumScale);
        void planeSweepUpdateBestPlane(const DeviceBuffer<float>& newCosts, int width, int height, int currPlaneIndex,
                                       DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<int>& bestPlanes);
        void planeSweepUpdateBestPlaneSubPixel(const DeviceBuffer<float>& currCosts, const DeviceBuffer<float>& prev1, const DeviceBuffer<float>& prev2, int width, int height, int prevPlaneIdx,
                                               DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets);
        void planeSweepUpdateBestAndSecondBestPlane(const DeviceBuffer<float>& newCosts, int width, int height, int currPlaneIndex,
                                                    DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<float>& secondBestPlaneCosts, DeviceBuffer<int>& bestPlanes);
        void planeSweepUpdateBestAndSecondBestPlaneSubPixel(const DeviceBuffer<float>& currCosts, const DeviceBuffer<float>& prev1, const DeviceBuffer<float>& prev2,
                                                            int width, int height, int prevPlaneIdx, DeviceBuffer<float>& bestPlaneCosts,
                                                            DeviceBuffer<float>& secondBestPlaneCosts, DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets);
        void planeSweepMinFloat(DeviceBuffer<float>& buf1, DeviceBuffer<float>& buf2, DeviceBuffer<float>& minBuf);
        void computeUniquenessRatio(DeviceBuffer<float>& bestCost, DeviceBuffer<float>& secondBestCost, DeviceBuffer<float>& uniquenessRatios);
        void planeSweepComputeBestDepths(DeviceBuffer<int>& bestPlanes, int numPlanes, vector<float>& planes,
                                         float* bestDepthsAddr, int bestDepthsPitch, vector<float> &KrefInv);
        void planeSweepComputeBestDepthsSubPixelInverse(DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& planeOffsets, int numPlanes, vector<float>& planes,
                                                 float* bestDepthsAddr, int bestDepthsPitch, vector<float> &KrefInv);
        void planeSweepComputeBestDepthsSubPixelDirect(DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& planeOffsets, int numPlanes, vector<float>& planes,
                                                       float* bestDepthsAddr, int bestDepthsPitch, vector<float> &KrefInv);
    }
}

namespace D3D
{
    enum PlaneSweepOcclusionMode
    {
        PLANE_SWEEP_OCCLUSION_NONE,
        PLANE_SWEEP_OCCLUSION_BEST_K,
        PLANE_SWEEP_OCCLUSION_REF_SPLIT
    };

    enum PlaneSweepPlaneGenerationMode
    {
        PLANE_SWEEP_PLANEMODE_UNIFORM_DEPTH,
        PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY
    };

    enum PlaneSweepMatchingCosts
    {
        PLANE_SWEEP_SAD,
        PLANE_SWEEP_ZNCC
    };

    struct CudaPlaneSweepImage
    {
        D3D_CUDA::DeviceImage devImg;
        CameraMatrix<double> cam;
    };

    enum PlaneSweepSubPixelInterpMode
    {
        PLANE_SWEEP_SUB_PIXEL_INTERP_DIRECT,
        PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE
    };

    class CudaPlaneSweep
    {
    public:
        CudaPlaneSweep();
        ~CudaPlaneSweep();
        int addImage(Mat image, CameraMatrix<double>& cam);
        int addDeviceImage(D3D_CUDA::DeviceImage& devImg, CameraMatrix<double>& cam);
        void deleteImage(int id);

        // needs to be set prior to run
        void setZRange(double nearZ, double farZ);
        void setMatchWindowSize(int w, int h);
        void setNumPlanes(int num);
        void setOcclusionMode(PlaneSweepOcclusionMode occlusionMode);
        void setOcclusionBestK(int bestK);
        void setPlaneGenerationMode(PlaneSweepPlaneGenerationMode planeGenerationMode);
        void setMatchingCosts(PlaneSweepMatchingCosts matchingCosts);
        void setScale(double scale);
        void updateCameraMatrix(int id, CameraMatrix<double>& cam);

        void enableOutputBestDepth(bool enabled = true);
        void enableColorMatching(bool enabled = true);
        void enableBoxFilterBeforeOcclusion(bool enabled = true);
        void enableOutputBestCosts(bool enabled = true);
        void enableOuputUniquenessRatio(bool enabled = true);
        void enableOutputCostVolume(bool enabled = true);
        void enableSubPixel(bool enabled = true);

        void setSubPixelInterpolationMode(PlaneSweepSubPixelInterpMode interpMode);

        DepthMap<float, double> getBestDepth();
        Grid<float> getBestCosts();
        Grid<float> getUniquenessRatios();
        Grid<float> getCostVolume();
        Grid<Vector4d> getPlanes();
        CameraMatrix<double> getRefImgCam();

        cv::Mat downloadImage(int id);

        void process(int refImgId);
        void process(int refImgId, Grid<Vector4d>& planes);

        void deallocateBuffers();

    private:
        map<int, CudaPlaneSweepImage> images;

        int nextId;

        double nearZ;
        double farZ;

        double scale;

        int matchWindowWidth;
        int matchWindowHeight;
        int matchWindowRadiusX;
        int matchWindowRadiusY;

        int numPlanes;

        PlaneSweepOcclusionMode occlusionMode;
        int occlusionBestK;

        bool boxFilterBeforeOcclusion;

        PlaneSweepPlaneGenerationMode planeGenerationMode;

        PlaneSweepMatchingCosts matchingCosts;

        void planeHomography(const Matrix<double, 4, 1>& plane, const CameraMatrix<double>& refCam, const CameraMatrix<double>& otherCam, float* H);
        void matchImage(const CudaPlaneSweepImage& refImg, const CudaPlaneSweepImage& otherImg, const Matrix<double, 4, 1>& plane, D3D_CUDA::DeviceBuffer<float>& costBuffer);
//        void calcSAD(const Mat& img1, const Mat& img2, Mat& costs);
//        void boxFilterMatchingWindow(const Mat& img, Mat& filteredImg);
//        void matchAccumNoOcc(const CudaPlaneSweepTextureImage& refImg, const CudaPlaneSweepTextureImage& otherImg, const Vector4& plane, float accumScale);
        double largestBaseline(const CudaPlaneSweepImage& refImg);

        // enabled features
        bool outputBestDepthEnabled;
        bool outputBestCostsEnabled;
        bool colorMatchingEnabled;
        bool outputUniquenessRatioEnabled;
        bool outputCostVolumeEnabled;
        bool subPixelEnabled;

        PlaneSweepSubPixelInterpMode subPixelInterpMode;

        // for output
        DepthMap<float, double> bestDepth;
        Grid<float> bestCosts;
        Grid<float> uniqunessRatios;
        Grid<float> costVolume;
        Grid<Vector4d> planes;
        CameraMatrix<double> refImgCam;

//        CudaPlaneSweepImage warpingBuffer;
//        CudaPlaneSweepFloatBuffer costBuffer;
        D3D_CUDA::DeviceBuffer<float> costAccumBuffer;
        D3D_CUDA::DeviceBuffer<float> subPixelCostAccumBufferPrev1;
        D3D_CUDA::DeviceBuffer<float> subPixelCostAccumBufferPrev2;
        D3D_CUDA::DeviceBuffer<float> subPixelPlaneOffsetsBuffer;
//        CudaPlaneSweepFloatBuffer costAccumBuffer;
        D3D_CUDA::DeviceBuffer<float>  boxFilterTempBuffer;
        D3D_CUDA::DeviceBuffer<int>  bestPlaneBuffer;
        D3D_CUDA::DeviceBuffer<float>  bestPlaneCostBuffer;
        D3D_CUDA::DeviceBuffer<float> secondBestPlaneCostBuffer;
        vector<D3D_CUDA::DeviceBuffer<float> > costBuffers;

        // Buffers for best K
        D3D_CUDA::DeviceBuffer<float>  bestKBuffer0;
        D3D_CUDA::DeviceBuffer<float>  bestKBuffer1;

        // Buffers for half sequence
        D3D_CUDA::DeviceBuffer<float>  costAccumBeforeBuffer;
        D3D_CUDA::DeviceBuffer<float>  costAccumAfterBuffer;

        // Buffers for NCC matching costs
        D3D_CUDA::DeviceBuffer<float>  refImgBoxFilterBuffer;
        D3D_CUDA::DeviceBuffer<float>  otherImgBoxFilterBuffer;
        D3D_CUDA::DeviceBuffer<float>  refImgSqrBoxFilterBuffer;
        D3D_CUDA::DeviceBuffer<float>  otherImgSqrBoxFilterBuffer;
        D3D_CUDA::DeviceBuffer<float>  imgProdBoxFilterBuffer;
        D3D_CUDA::DeviceBuffer<float>  boxFilterSqrTempBuffer;
        D3D_CUDA::DeviceBuffer<float>  boxFilterProdTempBuffer;
    };

}

#endif // PLANESWEEP_H
