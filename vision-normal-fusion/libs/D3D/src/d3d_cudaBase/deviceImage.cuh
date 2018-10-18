#ifndef DEVICEIMAGE_CUH
#define DEVICEIMAGE_CUH

inline __device__ unsigned char& D3D_CUDA::DeviceImage::operator()(unsigned int x, unsigned int y)
{
    return *(addr + y*pitch + x);
}

inline __device__ unsigned char& D3D_CUDA::DeviceImage::operator()(unsigned int x, unsigned int y, unsigned int c)
{
    return *(addr + y*pitch + x*numChannels + c);
}


#endif // DEVICEIMAGE_CUH
