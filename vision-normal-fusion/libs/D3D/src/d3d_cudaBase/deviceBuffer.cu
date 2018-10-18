#include "deviceBuffer.h"
#include "deviceBuffer.cuh"
#include <iostream>

namespace D3D_CUDA
{
    namespace DeviceBufferDeviceCode
    {
        template<typename T>
        __global__ void clearKernel(DeviceBuffer<T> buf, T value)
        {
            // get position of outupt
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < buf.getWidth() && y < buf.getHeight())
            {
                buf(x,y) = value;
            }
        }
    }
}

using namespace D3D;
using namespace D3D_CUDA;
using namespace DeviceBufferDeviceCode;


template<typename T>
DeviceBuffer<T>::DeviceBuffer()
{
    addr = 0;
}

template<typename T>
void DeviceBuffer<T>::allocatePitched(int width, int height)
{
//   // to avoid memory leaks
//   deallocate();

   D3D_CUDA_CHECKED_CALL( cudaMallocPitch(&addr, &pitch, width*sizeof(T), height); )
   this->width = width;
   this->height = height;
}

template<typename T>
void DeviceBuffer<T>::reallocatePitched(int width, int height)
{
    if (addr != 0)
    {
        if (width == this->width && height == this->height)
        {
            return;
        }
        deallocate();
    }
    allocatePitched(width, height);
}




template<typename T>
void DeviceBuffer<T>::deallocate()
{
//    if (addr != 0)
//    {
        D3D_CUDA_CHECKED_CALL( cudaFree((void *)addr); )
//    }
    addr = 0;
}

template<typename T>
void DeviceBuffer<T>::clear(T value)
{

    dim3 gridDim(getNumTiles(width, TILE_WIDTH), getNumTiles(height, TILE_HEIGHT));
    dim3 blockDim(TILE_WIDTH, TILE_HEIGHT);

    clearKernel<<<gridDim, blockDim>>>(*this, value);
}

template<typename T>
void DeviceBuffer<T>::upload(T* dataPtr, size_t dataPitch)
{
    // pitch is in bytes
    D3D_CUDA_CHECKED_CALL( cudaMemcpy2D(addr, pitch, dataPtr, dataPitch, width*sizeof(T), height, cudaMemcpyHostToDevice); )
}


#ifdef _MSC_VER
#pragma warning( disable : 4661)
#endif

// instantiate needed buffers
template class DeviceBuffer<float>;
template class DeviceBuffer<int>;

