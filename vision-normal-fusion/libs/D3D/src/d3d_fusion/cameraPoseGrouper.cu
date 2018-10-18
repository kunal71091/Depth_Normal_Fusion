#include <d3d_cudaBase/cudaCommon.h>
#include <cuda_gl_interop.h>
#include <iostream>

namespace D3D_CUDA
{
namespace CameraPoseGrouperDeviceCode
{

texture<uchar1, 2> bufTexture;
const int SUMMING_UP_NUM_THREADS = 16;


void setUpCudaInterop(GLuint bufferId, cudaGraphicsResource** bufRes, cudaArray** dArray, float** tempGlobMem)
{
    D3D_CUDA_CHECKED_CALL(cudaGraphicsGLRegisterImage(bufRes, bufferId,GL_RENDERBUFFER, cudaGraphicsRegisterFlagsReadOnly);)

    D3D_CUDA_CHECKED_CALL(cudaGraphicsMapResources(1, bufRes))

    D3D_CUDA_CHECKED_CALL(cudaGraphicsSubResourceGetMappedArray(dArray, *bufRes, 0, 0))

    // get array info
    struct cudaChannelFormatDesc channelDesc;
    struct cudaExtent extent;
    unsigned int flags;
    D3D_CUDA_CHECKED_CALL(cudaArrayGetInfo(&channelDesc, &extent, &flags, *dArray))

    D3D_CUDA_CHECKED_CALL(cudaMalloc(tempGlobMem, extent.height*sizeof(float)))

//    std::cout << "Array size: " << extent.width << ", " << extent.height << ", " << extent.depth << std::endl;
//    std::cout << "Array channel desc: " << channelDesc.x << ", " << channelDesc.y << ", " << channelDesc.z << ", " << channelDesc.w << ", " << channelDesc.f << std::endl;
}

void cleanUpCudaInterop(struct cudaGraphicsResource** bufRes, float** tempGlobMem)
{
    if (*bufRes != 0)
    {
        D3D_CUDA_CHECKED_CALL(cudaGraphicsUnmapResources(1, bufRes))
        *bufRes = 0;
    }

    if (*tempGlobMem != 0)
    {
        D3D_CUDA_CHECKED_CALL(cudaFree(*tempGlobMem))
        *tempGlobMem = 0;
    }
}__global__ void summingKernelHoriz(int width, int height, float* rowSums)
{
    // calculate row index
    unsigned int y = blockIdx.x*blockDim.x + threadIdx.x;

    if (y < height)
    {
        float sum = 0;
        for (int x = 0; x < width; x++)
        {
            uchar1 val = tex2D(bufTexture, x, y);
            if (val.x > 0)
                sum ++;
        }
        rowSums[y] = sum;
    }
}

__global__ void summingKernelPass1(int length, float* values)
{
    __shared__ float valuesShared[SUMMING_UP_NUM_THREADS];

    const int y = blockIdx.x*blockDim.x + threadIdx.x;

    if (y < length)
    {
        valuesShared[threadIdx.x] = values[y];
    }
    else
    {
        valuesShared[threadIdx.x] = 0;
    }

    __syncthreads();

    if (threadIdx.x == 0)
    {
        float sum = 0;
        for (int i = 0; i < SUMMING_UP_NUM_THREADS; i++)
        {
            sum += valuesShared[i];
        }
        values[blockIdx.x*blockDim.x] = sum;
    }
}

__global__ void summingKernelPass2(int length, float* values)
{
    if (threadIdx.x == 0)
    {
        const int skip = blockDim.x;
        float sum = 0;
        for (int i = 0; i < length; i+=skip)
        {
            sum += values[i];
        }
        values[0] = sum;
    }
}

float sumUpBuffer(struct cudaArray* dArray, float* rowSums)
{
    struct cudaChannelFormatDesc channelDesc;
    struct cudaExtent extent;
    unsigned int flags;
    D3D_CUDA_CHECKED_CALL(cudaArrayGetInfo(&channelDesc, &extent, &flags, dArray))

    // bind the texture
    D3D_CUDA_CHECKED_CALL(cudaBindTextureToArray(bufTexture, dArray, channelDesc))

    dim3 gridDim(extent.height % SUMMING_UP_NUM_THREADS == 0 ? extent.height/SUMMING_UP_NUM_THREADS : extent.height/SUMMING_UP_NUM_THREADS + 1);
    dim3 blockDim(SUMMING_UP_NUM_THREADS);

    // Some fancy way of summing everything,
    // I have no clue if it is actually fast
    summingKernelHoriz<<<gridDim, blockDim>>>(extent.width, extent.height, rowSums);
    D3D_CUDA_CHECK_ERROR

    // first pass of the final summing
    summingKernelPass1<<<gridDim, blockDim>>>(extent.height, rowSums);
    D3D_CUDA_CHECK_ERROR

    // finally sum up
    summingKernelPass2<<<1, blockDim>>>(extent.height, rowSums);
    D3D_CUDA_CHECK_ERROR

    float sum;
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(&sum, rowSums, sizeof(float), cudaMemcpyDeviceToHost);)

    // unbind the texture
    D3D_CUDA_CHECKED_CALL(cudaUnbindTexture(bufTexture))

    return sum;
}
}
}
