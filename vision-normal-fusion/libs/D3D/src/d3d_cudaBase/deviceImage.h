#ifndef DEVICEIMAGE_H
#define DEVICEIMAGE_H

#include "cudaCommon.h"
#include "cuda_runtime.h"
//#include <opencv2/core/core.hpp>
//#include <string>

namespace cv
{
 class Mat;
}


namespace D3D_CUDA
{
    class DeviceImage
    {
    public:
        DeviceImage();

        // host and device functions
        inline __host__ __device__ unsigned char* getAddr() const
        {
            return addr;
        }
        inline __host__ __device__ int getWidth() const
        {
            return width;
        }
        inline __host__ __device__ int getHeight() const
        {
            return height;
        }
        inline __host__ __device__ int getNumChannels() const
        {
            return numChannels;
        }
        inline __host__ __device__ int getPitch() const
        {
            return (int) pitch;
        }
        inline __host__ __device__ bool isAllocated() const
        {
            return addr != 0;
        }
        inline __host__ __device__ int2 getSize() const
        {
            return make_int2(width, height);
        }

        // device only functions
        inline __device__ unsigned char& operator()(unsigned int x, unsigned int y);
        inline __device__ unsigned char& operator()(unsigned int x, unsigned int y, unsigned int c);

        // host only functions
        void allocatePitchedAndUpload(const cv::Mat& img);
        void reallocatePitchedAndUpload(const cv::Mat& img);
        void allocatePitched(int width, int height, int numChannels);
        void download(cv::Mat& img);
        void clear(unsigned char value);
        void deallocate();

    private:
        unsigned char* addr;
        int width;
        int height;
        int numChannels;
        size_t pitch;
    };
}

#endif // CUDADEVICEIMAGE_H
