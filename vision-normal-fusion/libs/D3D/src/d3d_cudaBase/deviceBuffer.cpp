#include "deviceBuffer.h"

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using namespace D3D_CUDA;

template <typename T>
void DeviceBuffer<T>::downloadAndDisplay(int waitTime, T minVal, T maxVal, std::string windowTitle)
{
    cv::Mat_<T> mat(height, width);

    download((T*)mat.data, mat.step);

    mat -= minVal;
    mat /= (maxVal - minVal);

    cv::imshow(windowTitle.c_str(), mat);

    cv::waitKey(waitTime);
}

template<typename T>
void DeviceBuffer<T>::download(T* dstPtr, size_t dstPitch)
{
    D3D_CUDA_CHECKED_CALL( cudaMemcpy2D(dstPtr, dstPitch, addr, pitch, sizeof(T)*width, height, cudaMemcpyDeviceToHost); )
}

#ifdef _MSC_VER
#pragma warning( disable : 4661)
#endif

template class DeviceBuffer<float>;
template class DeviceBuffer<int>;