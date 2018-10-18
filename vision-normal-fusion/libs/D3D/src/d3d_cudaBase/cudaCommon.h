#ifndef CUDACOMMON_H
#define CUDACOMMON_H

#include <sstream>

#include <d3d_base/exception.h>

using std::ostringstream;


namespace D3D_CUDA
{

    const int TILE_WIDTH = 16;
    const int TILE_HEIGHT = 16;

    inline int getNumTiles(int totalSize, int tileSize)
    {
        const int div = totalSize/tileSize;
        return totalSize % tileSize == 0 ? div : div + 1;
    }

#ifdef _MSC_VER

    #define D3D_CUDA_CHECKED_CALL(cuda_call)                                                    \
    {                                                                                       \
        cudaError err = cuda_call;                                                          \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw D3D::Exception(__FILE__, __LINE__, __FUNCSIG__, os.str().c_str());     \
        }                                                                                   \
    }

    #define D3D_CUDA_CHECK_ERROR                                                                \
    {                                                                                       \
        cudaError err = cudaGetLastError();                                                 \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw D3D::Exception(__FILE__, __LINE__, __FUNCSIG__, os.str().c_str());     \
        }                                                                                   \
    }

#else
    #define D3D_CUDA_CHECKED_CALL(cuda_call)                                                    \
    {                                                                                       \
        cudaError err = cuda_call;                                                          \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str());     \
        }                                                                                   \
    }

    #define D3D_CUDA_CHECK_ERROR                                                                \
    {                                                                                       \
        cudaError err = cudaGetLastError();                                                 \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str());     \
        }                                                                                   \
    }
#endif
}




#endif //CUDACOMMON_H
