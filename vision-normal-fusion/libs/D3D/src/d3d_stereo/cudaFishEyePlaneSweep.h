#ifndef FISHEYECUDAPLANESWEEP_H
#define FISHEYECUDAPLANESWEEP_H

#include <d3d_base/fishEyeCameraMatrix.h>
#include <d3d_cudaBase/deviceImage.h>
#include <d3d_base/grid.h>
#include <d3d_base/fishEyeDepthMap.h>
#include <d3d_cudaBase/deviceBuffer.h>

#include <opencv2/core/core.hpp>

#include <map>
#include <vector>

namespace D3D_CUDA
{
    namespace CudaFishEyePlaneSweepDeviceCode
    {
        // device function declarations
        void planeSweepInitTexturing();

        void planeSweepBoxFilterCosts(DeviceBuffer<float>& costBuf, DeviceBuffer<float>& filteredCostBuf, int radius_x, int radius_y);
        void planeSweepBoxFilterImageAndSqrImage(const DeviceImage& refImg,
                                                 DeviceBuffer<float>& boxFilterBuf, DeviceBuffer<float>& boxFilterSqrBuf,
                                                 DeviceBuffer<float>& tempBuf, DeviceBuffer<float>& tempSqrBuf, int radius_x, int radius_y);
        void planeSweepWarpZNCC(const DeviceImage& otherImg, float* RT,
                                const float* KRefInv, const float XiRef,
                                const float* Kother, const float XiOther,
                                const DeviceImage& refImg,
                                DeviceBuffer<float>& refImgBoxFilterBuf, DeviceBuffer<float>& refImgSqrBoxFilterBuf,
                                DeviceBuffer<float>& costBuf, int radius_x, int radius_y);
        void planeSweepWarpZNCCAccum(const DeviceImage& otherImg, float* RT,
                                     const float* KRefInv, const float XiRef,
                                     const float* Kother, const float XiOther,
                                     const DeviceImage& refImg,
                                     DeviceBuffer<float>& refImgBoxFilterBuf, DeviceBuffer<float>& refImgSqrBoxFilterBuf,
                                     float accumScale, DeviceBuffer<float>& costAccumBuf, int radius_x, int radius_y);
        void planeSweepWarpAD(const DeviceImage& srcImg, const float* RT,
                              const float* KRefInv, const float XiRef,
                              const float* Kother, const float XiOther,
                              const DeviceImage& refImg,
                              DeviceBuffer<float>& costBuf);
        void planeSweepWarpADAccum(DeviceImage& srcImg, const float* RT,
                                   const float* KRefInv, const float XiRef,
                                   const float* Kother, const float XiOther,
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
                                                            int width, int height, int prevPlaneIdx, DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<float>& secondBestPlaneCosts, DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets);
        void planeSweepMinFloat(DeviceBuffer<float>& buf1, DeviceBuffer<float>& buf2, DeviceBuffer<float>& minBuf);
        void computeUniquenessRatio(DeviceBuffer<float>& bestCost, DeviceBuffer<float>& secondBestCost, DeviceBuffer<float>& uniquenessRatios);
        void planeSweepComputeBestDepths(DeviceBuffer<int>& bestPlanes, int numPlanes, std::vector<float>& planes,
                                         float* bestDepthsAddr, int bestDepthsPitch, std::vector<float> &KrefInv, float XiRef);
        void planeSweepComputeBestDepthsSubPixelInverse(DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets, int numPlanes, std::vector<float>& planes,
                                                        float* bestDepthsAddr, int bestDepthsPitch, std::vector<float> &KrefInv, float XiRef);
        void planeSweepComputeBestDepthsSubPixelDirect(DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& planeOffsets, int numPlanes, std::vector<float>& planes,
                                                       float* bestDepthsAddr, int bestDepthsPitch, std::vector<float> &KrefInv, float XiRef);
    }
}


namespace D3D
{
    enum FishEyePlaneSweepOcclusionMode
    {
        FISH_EYE_PLANE_SWEEP_OCCLUSION_NONE,
        FISH_EYE_PLANE_SWEEP_OCCLUSION_BEST_K,
        FISH_EYE_PLANE_SWEEP_OCCLUSION_REF_SPLIT
    };

    enum FishEyePlaneSweepPlaneGenerationMode
    {
        FISH_EYE_PLANE_SWEEP_PLANEMODE_UNIFORM_DEPTH,
        FISH_EYE_PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY
    };

    enum FishEyePlaneSweepMatchingCosts
    {
        FISH_EYE_PLANE_SWEEP_SAD,
        FISH_EYE_PLANE_SWEEP_ZNCC
    };

    struct CudaFishEyePlaneSweepImage
    {
        D3D_CUDA::DeviceImage devImg;
        FishEyeCameraMatrix<double> cam;
    };

    enum FishEyePlaneSweepSubPixelInterpMode
    {
        FISH_EYE_PLANE_SWEEP_SUB_PIXEL_INTERP_DIRECT,
        FISH_EYE_PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE
    };

    class CudaFishEyePlaneSweep
    {
    public:
        CudaFishEyePlaneSweep();
        ~CudaFishEyePlaneSweep();

        int addImage(cv::Mat image, FishEyeCameraMatrix<double>& cam);
        int addDeviceImage(D3D_CUDA::DeviceImage&, FishEyeCameraMatrix<double>& cam);
        void updateCameraMatrix(int id, FishEyeCameraMatrix<double>& cam);
        void deleteImage(int id);

        // needs to be set prior to run
        void setZRange(double nearZ, double farZ);
        void setMatchWindowSize(int w, int h);
        void setNumPlanes(int num);
        void setOcclusionMode(FishEyePlaneSweepOcclusionMode occlusionMode);
        void setOcclusionBestK(int bestK);
        void setPlaneGenerationMode(FishEyePlaneSweepPlaneGenerationMode planeGenerationMode);
        void setMatchingCosts(FishEyePlaneSweepMatchingCosts matchingCosts);
        void setScale(double scale);
        void setSubPixelInterpolationMode(FishEyePlaneSweepSubPixelInterpMode interpMode);

        void enableOutputBestDepth(bool enabled = true);
        void enableBoxFilterBeforeOcclusion(bool enabled = true);
        void enableOutputBestCosts(bool enabled = true);
        void enableOuputUniquenessRatio(bool enabled = true);
        void enableOutputCostVolume(bool enabled = true);
        void enableOutputPlanes(bool enabled = true);
        void enableSubPixel(bool enabled = true);

        FishEyeDepthMap<float, double> getBestDepth();
        Grid<float> getBestCosts();
        Grid<float> getUniquenessRatios();
        Grid<float> getCostVolume();
        Grid<Eigen::Vector4d> getPlanes();
        FishEyeCameraMatrix<double> getRefImgCam();

        cv::Mat downloadImage(int id);

        void process(int refImgId);
        void process(int refImgId, Grid<Eigen::Vector4d>& planes);

        void deallocateBuffers();

    private:
        std::map<int, CudaFishEyePlaneSweepImage> images;

        int nextId;

        double nearZ;
        double farZ;

        double scale;

        int matchWindowWidth;
        int matchWindowHeight;
        int matchWindowRadiusX;
        int matchWindowRadiusY;

        int numPlanes;

        FishEyePlaneSweepOcclusionMode occlusionMode;
        int occlusionBestK;

        bool boxFilterBeforeOcclusion;

        FishEyePlaneSweepPlaneGenerationMode planeGenerationMode;

        FishEyePlaneSweepMatchingCosts matchingCosts;

        void planeRT(const Eigen::Matrix<double, 4, 1>& plane, const FishEyeCameraMatrix<double>& refCam, const FishEyeCameraMatrix<double>& otherCam, float* RT);
        void eigenMat3dToInlineArryf(const Eigen::Matrix3d& mat, float* arr);
        void matchImage(const CudaFishEyePlaneSweepImage& refImg,
                         const CudaFishEyePlaneSweepImage& otherImg,
                         const Eigen::Matrix<double, 4, 1>& plane, D3D_CUDA::DeviceBuffer<float>& costBuffer);

         // enabled features
         bool outputBestDepthEnabled;
         bool outputBestCostsEnabled;
         bool outputUniquenessRatioEnabled;
         bool outputCostVolumeEnabled;
         bool subPixelEnabled;

         FishEyePlaneSweepSubPixelInterpMode subPixelInterpMode;

         // for output
         FishEyeDepthMap<float, double> bestDepth;
         Grid<float> bestCosts;
         Grid<float> uniqunessRatios;
         Grid<float> costVolume;
         Grid<Eigen::Vector4d> planes;
         FishEyeCameraMatrix<double> refImgCam;

         D3D_CUDA::DeviceBuffer<float> costAccumBuffer;
         D3D_CUDA::DeviceBuffer<float> subPixelCostAccumBufferPrev1;
         D3D_CUDA::DeviceBuffer<float> subPixelCostAccumBufferPrev2;
         D3D_CUDA::DeviceBuffer<float> subPixelPlaneOffsetsBuffer;
         D3D_CUDA::DeviceBuffer<float>  boxFilterTempBuffer;
         D3D_CUDA::DeviceBuffer<int>  bestPlaneBuffer;
         D3D_CUDA::DeviceBuffer<float>  bestPlaneCostBuffer;
         D3D_CUDA::DeviceBuffer<float> secondBestPlaneCostBuffer;
         std::vector<D3D_CUDA::DeviceBuffer<float> > costBuffers;

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



#endif //FISHEYECUDAPLANESWEEP_H


