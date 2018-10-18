#include "deviceImage.h"


#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using namespace D3D_CUDA;

void DeviceImage::allocatePitchedAndUpload(const cv::Mat& img)
{
    if (img.type() == CV_8UC4)
    {
        this->width = img.cols;
        this->height = img.rows;
        this->numChannels = 4;
    }
    else if (img.type() == CV_8UC1)
    {
        this->width = img.cols;
        this->height = img.rows;
        this->numChannels = 1;
    }
    else
    {
        D3D_THROW_EXCEPTION ( "Only BRGA and Grayscale supported!")
    }

    D3D_CUDA_CHECKED_CALL( cudaMallocPitch(&addr, &pitch, width*numChannels, height); )
    D3D_CUDA_CHECKED_CALL( cudaMemcpy2D(addr, pitch, img.data, img.step, width*numChannels, height, cudaMemcpyHostToDevice); )
}

void DeviceImage::reallocatePitchedAndUpload(const cv::Mat& img)
{
    if ((img.type() == CV_8UC4 && numChannels != 4)
            || (img.type() == CV_8UC1 && numChannels != 1)
            || (img.cols != width)
            || (img.rows != height))
    {
        deallocate();
        allocatePitchedAndUpload(img);
    }
    else
    {
        D3D_CUDA_CHECKED_CALL( cudaMemcpy2D(addr, pitch, img.data, img.step, width*numChannels, height, cudaMemcpyHostToDevice); )
    }
}

void DeviceImage::allocatePitched(int width, int height, int numChannels)
{
    this->width = width;
    this->height = height;
    this->numChannels = numChannels;

    D3D_CUDA_CHECKED_CALL( cudaMallocPitch(&addr, &pitch, width*numChannels, height); )
}

void DeviceImage::download(cv::Mat& img)
{
    if (numChannels == 4)
    {
        img = cv::Mat(height, width, CV_8UC4);
    }
    else if (numChannels == 1)
    {
        img = cv::Mat(height, width, CV_8UC1);
    }
    else
    {
        D3D_THROW_EXCEPTION ( "Only BRGA and BRG supported!")
    }
    D3D_CUDA_CHECKED_CALL( cudaMemcpy2D(img.data, img.step, addr, pitch, width*numChannels, height, cudaMemcpyDeviceToHost); )
}
