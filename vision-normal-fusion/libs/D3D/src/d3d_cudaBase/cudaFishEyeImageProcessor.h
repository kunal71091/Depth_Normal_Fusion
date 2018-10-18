#ifndef CUDAFISHEYEIMAGEPROCESSOR_H
#define CUDAFISHEYEIMAGEPROCESSOR_H

#include "deviceImage.h"
#include <d3d_base/fishEyeCameraMatrix.h>

namespace D3D_CUDA
{
    namespace CudaFishEyeImageProcessorDeviceCode
    {
        void fishEyeImageProcessorInitTexturing();
        void fishEyeImageProcessorUndistort(DeviceImage& inputImg, DeviceImage& outputImg, double k1_input,
                                            double k2_input, double p1_input, double p2_input,
                                            double k11_input, double k13_input, double k22_input, double k23_input,
                                            double k11inv_output, double k13inv_output, double k22inv_output, double k23inv_output);
        void fishEyeImageProcessorUndistortRectify(DeviceImage& inputImg, DeviceImage& outputImg, double xi_input, double k1_input,
                                                   double k2_input, double p1_input, double p2_input,
                                                   double k11_input, double k13_input, double k22_input, double k23_input,
                                                   double k11inv_output, double k13inv_output, double k22inv_output, double k23inv_output);
    }

    class CudaFishEyeImageProcessor
    {
    public:
        CudaFishEyeImageProcessor();
        void setInputImg(DeviceImage& inputImg, D3D::FishEyeCameraMatrix<double>& camera);
        std::pair<DeviceImage, D3D::FishEyeCameraMatrix<double> > undistort(double iScale, double fScale, double k1, double k2, double p1, double p2);
        DeviceImage extractPinhole(double iScale, Eigen::Matrix3d& Kpinhole, double k1, double k2, double p1, double p2);
    private:
        DeviceImage inputImg;
        D3D::FishEyeCameraMatrix<double> camera;
    };
}

#endif //CUDAFISHEYEIMAGEPROCESSOR_H
