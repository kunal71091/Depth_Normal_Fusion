#include "deviceImage.h"
#include "deviceImage.cuh"


namespace D3D_CUDA
{
    namespace DeviceImageDeviceCode
    {
        __global__ void clearKernel(DeviceImage devImg, unsigned char value)
        {
            // get position of outupt
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < devImg.getWidth()*devImg.getNumChannels() && y < devImg.getHeight())
            {
                devImg(x,y) = value;
            }
        }
    }
}

using namespace D3D;
using namespace D3D_CUDA;
using namespace DeviceImageDeviceCode;


DeviceImage::DeviceImage()
{
    addr = 0;
}



void DeviceImage::deallocate()
{
    D3D_CUDA_CHECKED_CALL( cudaFree((void *)addr); )
    addr = 0;
}

void DeviceImage::clear(unsigned char value)
{

    dim3 gridDim(getNumTiles(width*numChannels, TILE_WIDTH), getNumTiles(height, TILE_HEIGHT));
    dim3 blockDim(TILE_WIDTH, TILE_HEIGHT);

    clearKernel<<<gridDim, blockDim>>>(*this, value);
}

