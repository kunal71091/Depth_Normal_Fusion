#include "cudaFishEyeImageProcessor.h"
#include <d3d_base/common.h>
#include <cmath>

using namespace D3D_CUDA;
using namespace D3D;

CudaFishEyeImageProcessor::CudaFishEyeImageProcessor()
{
    CudaFishEyeImageProcessorDeviceCode::fishEyeImageProcessorInitTexturing();
}

void CudaFishEyeImageProcessor::setInputImg(DeviceImage& inputImg, FishEyeCameraMatrix<double>& camera)
{
    this->inputImg = inputImg;
    this->camera = camera;
    if (camera.getK()(0,1) != 0)
    {
        D3D_THROW_EXCEPTION("Only Ks without skew allowed.")
    }
}

std::pair<DeviceImage, D3D::FishEyeCameraMatrix<double> > CudaFishEyeImageProcessor::undistort(double iScale, double fScale, double k1, double k2, double p1, double p2)
{
    if (inputImg.getNumChannels() != 1)
    {
        D3D_THROW_EXCEPTION("Only grayscale supported.")
    }

    DeviceImage outputImage;
    int width = (int) round(iScale*inputImg.getWidth());
    int height = (int) round(iScale*inputImg.getHeight());

    outputImage.allocatePitched(width, height, 1);

    Eigen::Matrix3d Knew = camera.getK();
    Knew /= Knew(2,2); // make sure it is normalized
    Knew(0,0) *= iScale*fScale;
    Knew(1,1) *= iScale*fScale;
    Knew(0,2) = (Knew(0,2) + 0.5)*iScale - 0.5;
    Knew(1,2) = (Knew(1,2) + 0.5)*iScale -0.5;


    if (Knew(0,1) != 0)
    {
        D3D_THROW_EXCEPTION("Only Ks without skew allowed.")
    }

    Eigen::Matrix3d K = camera.getK();
    K /= K(2,2);

    Eigen::Matrix3d KnewInv = Knew.inverse();


    FishEyeCameraMatrix<double> newCamera(Knew, camera.getR(), camera.getT(), camera.getXi());


    CudaFishEyeImageProcessorDeviceCode::fishEyeImageProcessorUndistort(inputImg, outputImage, k1, k2, p1, p2,
                                                                        K(0,0), K(0,2), K(1,1), K(1,2), KnewInv(0,0), KnewInv(0,2), KnewInv(1,1), KnewInv(1,2));

    return std::make_pair<DeviceImage, FishEyeCameraMatrix<double> >(outputImage, newCamera);
}

DeviceImage CudaFishEyeImageProcessor::extractPinhole(double iScale, Eigen::Matrix3d& KPinhole, double k1, double k2, double p1, double p2)
{
    if (inputImg.getNumChannels() != 1)
    {
        D3D_THROW_EXCEPTION("Only grayscale supported.")
    }

    DeviceImage outputImage;
    int width = (int) round(iScale*inputImg.getWidth());
    int height = (int) round(iScale*inputImg.getHeight());

    outputImage.allocatePitched(width, height, 1);

    KPinhole /= KPinhole(2,2);

    if (KPinhole(0,1) != 0)
    {
        D3D_THROW_EXCEPTION("Only Ks without skew allowed.")
    }

    Eigen::Matrix3d KInvPinhole = KPinhole.inverse();

    Eigen::Matrix3d K = camera.getK();
    K /= K(2,2);

    CudaFishEyeImageProcessorDeviceCode::fishEyeImageProcessorUndistortRectify(inputImg, outputImage, camera.getXi(), k1, k2, p1, p2,
                                                                               K(0,0), K(0,2), K(1,1), K(1,2), KInvPinhole(0,0), KInvPinhole(0,2), KInvPinhole(1,1), KInvPinhole(1,2));

    return outputImage;
}

