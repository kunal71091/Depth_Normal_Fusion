#include <d3d_cudaBase/cudaCommon.h>
#include <iostream>

namespace D3D_CUDA
{
namespace CudaDepthIntegratorMultiClassv2DeviceCode
{


void initTile(int xRes, int yRes, int zRes, int numClasses, float** voxelsD)
{
    const int numBytes = xRes*yRes*zRes*numClasses*sizeof(float);
    D3D_CUDA_CHECKED_CALL(cudaMalloc(voxelsD, numBytes);)
    D3D_CUDA_CHECKED_CALL(cudaMemset(*voxelsD, 0, numBytes);)
}

void releaseDeviceMemory(void *addr)
{
    D3D_CUDA_CHECKED_CALL(cudaFree(addr);)
}

void allocateAndUploadTransformation(float* transformationH, float** transformationD)
{
    D3D_CUDA_CHECKED_CALL(cudaMalloc(transformationD, 4*4*sizeof(float));)
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*transformationD, transformationH, 4*4*sizeof(float), cudaMemcpyHostToDevice);)
}

void allocateAndUploadProjection(float* projectionH, float** projectionD)
{
    D3D_CUDA_CHECKED_CALL(cudaMalloc(projectionD, 3*4*sizeof(float));)
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*projectionD, projectionH, 3*4*sizeof(float), cudaMemcpyHostToDevice);)
}

void allocateAndUploadDepthData(float* depthDataH, int rows, int cols, float** depthDataD)
{
    D3D_CUDA_CHECKED_CALL(cudaMalloc(depthDataD, rows*cols*sizeof(float));)
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*depthDataD, depthDataH, rows*cols*sizeof(float), cudaMemcpyHostToDevice);)
}

void allocateAndUploadClassScoresData(float* classScoresDataH, int rows, int cols, int numClasses, float** classScoresDataD)
{
    D3D_CUDA_CHECKED_CALL(cudaMalloc(classScoresDataD, rows*cols*numClasses*sizeof(float)); )
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*classScoresDataD, classScoresDataH, rows*cols*numClasses*sizeof(float), cudaMemcpyHostToDevice);)

}


void downloadTile(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH)
{
    const int numBytes = xRes*yRes*zRes*sizeof(float);
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(voxelsH, voxelsD, numBytes, cudaMemcpyDeviceToHost);)
}

__device__ void project(float* projMat, float* point, float* projPoint)
{
    projPoint[0] = projMat[0]*point[0] + projMat[1]*point[1] + projMat[2]*point[2] + projMat[3];
    projPoint[1] = projMat[4]*point[0] + projMat[5]*point[1] + projMat[6]*point[2] + projMat[7];
    projPoint[2] = projMat[8]*point[0] + projMat[9]*point[1] + projMat[10]*point[2] + projMat[11];
}

__device__ void transform(float* transformMat, float* point, float* transfPoint)
{
    transfPoint[0] = transformMat[0]*point[0] + transformMat[1]*point[1] + transformMat[2]*point[2] + transformMat[3];
    transfPoint[1] = transformMat[4]*point[0] + transformMat[5]*point[1] + transformMat[6]*point[2] + transformMat[7];
    transfPoint[2] = transformMat[8]*point[0] + transformMat[9]*point[1] + transformMat[10]*point[2] + transformMat[11];
}

__global__ void integrationKernel(float* voxelD, const float* boxToGlobalD,
                             int xRes, int yRes, int zRes,
                             int numClasses, int freeSpaceClass,
                             float xBegin, float yBegin, float zBegin,
                             float deltaX, float deltaY, float deltaZ,
                             const float* classScoresDataD, const float* depthDataD, int rows, int cols,
                             float maxDepth, float minDepth,
                             float epsilon, float eta, float uncertFact, float rho,
                             const float* projD)
{
    __shared__ float proj[12];
    __shared__ float boxToGlobal[16];

    if (threadIdx.x < 12)
    {
        proj[threadIdx.x] = projD[threadIdx.x];
    }
    if (threadIdx.x < 16)
    {
        boxToGlobal[threadIdx.x] = boxToGlobalD[threadIdx.x];
    }
    __syncthreads();

    const int x = blockIdx.x;
    const int y = blockIdx.y;
    const int z = threadIdx.x;

    // voxel center in box coordinate frame
    float voxCenterPoint[3];
    voxCenterPoint[0] = xBegin + x*deltaX;
    voxCenterPoint[1] = yBegin + y*deltaY;
    voxCenterPoint[2] = zBegin + z*deltaZ;

    const float voxelDiameter = sqrtf(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);

    float voxCenterPointG[3];

    transform(boxToGlobal, voxCenterPoint, voxCenterPointG);

    float voxCenterProj[3];
    project(proj, voxCenterPointG, voxCenterProj);

    if (voxCenterProj[2] >= minDepth && voxCenterProj[2] <= maxDepth)
    {

        // perspective division
        int xp,yp;
        xp = round(voxCenterProj[0]/voxCenterProj[2]);
        yp = round(voxCenterProj[1]/voxCenterProj[2]);

        // test if inside image
        if (xp >= 0 && xp < cols && yp >= 0 && yp < rows)
        {

            const int depthMapIdx = yp*cols + xp;

            float depth = depthDataD[depthMapIdx];

// all transition approach

            int voxInd = (z*xRes*yRes + y*xRes + x)*numClasses;

            float bestScore = classScoresDataD[depthMapIdx*numClasses];
            int bestClass = 0;

            for (int i = 1; i < numClasses; i++)
            {
                if (classScoresDataD[depthMapIdx*numClasses + i] > bestScore)
                {
                    bestScore = classScoresDataD[depthMapIdx*numClasses + i];
                    bestClass = i;
                }
            }

            if (bestClass == freeSpaceClass)
            {
                voxelD[voxInd + freeSpaceClass] -= eta;
            }
            else
            {
                if (depth > 0)
                {

                    float dist = voxCenterProj[2] - depth;

                    // inside band
                    if (fabs(dist) < epsilon)
                    {
                        if (dist < 0)
                        {
                            // in front of surface
                            voxelD[voxInd + freeSpaceClass] -= 1;

                        }
                        else
                        {
                            // behind surface
                            voxelD[voxInd + bestClass] -= 1;
                        }
                    }
                }
            }
        }
    }



}

void integrateDepthMap(float* voxelD, const float* boxToGlobalD,
                       int xRes, int yRes, int zRes,
                       int numClasses, int freeSpaceClass,
                       float xBegin, float yBegin, float zBegin,
                       float deltaX, float deltaY, float deltaZ,
                       const float* classScoresDataD, const float* depthDataD, int rows, int cols,
                       float maxDepth, float minDepth,
                       float epsilon, float eta, float uncertFact, float rho,
                       const float* projD)
{
    dim3 dimGrid(xRes, yRes);
    dim3 dimBlock(zRes);

    std::cout << "Eta = " << eta << std::endl;

    integrationKernel<<<dimGrid,dimBlock>>>(voxelD, boxToGlobalD,
                                            xRes, yRes, zRes,
                                            numClasses, freeSpaceClass,
                                            xBegin, yBegin, zBegin,
                                            deltaX, deltaY, deltaZ,
                                            classScoresDataD, depthDataD, rows, cols,
                                            maxDepth, minDepth,
                                            epsilon, eta, uncertFact, rho,
                                            projD);
    D3D_CUDA_CHECK_ERROR

}

}
}

