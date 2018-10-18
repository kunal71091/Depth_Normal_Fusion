#ifndef DEVICEBUFFER_CUH
#define DEVICEBUFFER_CUH

template <typename T>
inline __device__ T& D3D_CUDA::DeviceBuffer<T>::operator()(unsigned int x, unsigned int y)
{
    return *((T*)((char*)addr + y*pitch) + x);
}


#endif // DEVICEBUFFER_CUH
