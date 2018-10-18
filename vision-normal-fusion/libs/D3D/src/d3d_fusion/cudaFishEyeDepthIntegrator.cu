#include <d3d_cudaBase/cudaCommon.h>
#include <iostream>

namespace D3D_CUDA
{
namespace CudaFishEyeDepthIntegratorDeviceCode
{


void initTile(int xRes, int yRes, int zRes, float** voxelsD)
{
    const int numBytes = xRes*yRes*zRes*sizeof(float);
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

void allocateAndUploadProjection(float* rMatrixH, float** rMatrixD, float* tVectorH, float** tVectorD, float* kMatrixH, float** kMatrixD)
{
    D3D_CUDA_CHECKED_CALL(cudaMalloc(rMatrixD, 3*3*sizeof(float));)
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*rMatrixD, rMatrixH, 3*3*sizeof(float), cudaMemcpyHostToDevice);)
    D3D_CUDA_CHECKED_CALL(cudaMalloc(tVectorD, 3*sizeof(float));)
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*tVectorD, tVectorH, 3*sizeof(float), cudaMemcpyHostToDevice);)
    D3D_CUDA_CHECKED_CALL(cudaMalloc(kMatrixD, 3*3*sizeof(float));)
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*kMatrixD, kMatrixH, 3*3*sizeof(float), cudaMemcpyHostToDevice);)
}

void allocateAndUploadDepthData(float* depthDataH, int rows, int cols, float** depthDataD)
{
    D3D_CUDA_CHECKED_CALL(cudaMalloc(depthDataD, rows*cols*sizeof(float));)
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(*depthDataD, depthDataH, rows*cols*sizeof(float), cudaMemcpyHostToDevice);)
}

void downloadTile(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH)
{
    const int numBytes = xRes*yRes*zRes*sizeof(float);
    D3D_CUDA_CHECKED_CALL(cudaMemcpy(voxelsH, voxelsD, numBytes, cudaMemcpyDeviceToHost);)
}

__device__ void transform(float* transformMat, float* point, float* transfPoint)
{
    transfPoint[0] = transformMat[0]*point[0] + transformMat[1]*point[1] + transformMat[2]*point[2] + transformMat[3];
    transfPoint[1] = transformMat[4]*point[0] + transformMat[5]*point[1] + transformMat[6]*point[2] + transformMat[7];
    transfPoint[2] = transformMat[8]*point[0] + transformMat[9]*point[1] + transformMat[10]*point[2] + transformMat[11];
}

__global__ void integrationKernelWithFixedWeights(float* voxelD, float* boxToGlobalD,
                                                  int xRes, int yRes, int zRes,
                                                  float xBegin, float yBegin, float zBegin,
                                                  float deltaX, float deltaY, float deltaZ,
                                                  float* depthDataD, int rows, int cols,
                                                  float maxDepth, float minDepth,
                                                  float epsilon, float eta, float delta1, float delta2, float delta3,
                                                  float* rMatD, float* tVecD, float* kMatD, float xi)
{
    __shared__ float R[9];
    __shared__ float K[9];
    __shared__ float T[3];
    __shared__ float boxToGlobal[16];


    if (threadIdx.x < 9)
    {
        R[threadIdx.x] = rMatD[threadIdx.x];
    }
    else if (threadIdx.x < 12)
    {
        T[threadIdx.x-9] = tVecD[threadIdx.x-9];
    }
    if (threadIdx.x < 9)
    {
        K[threadIdx.x] = kMatD[threadIdx.x];
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

    float voxCenterPointT[3];

    transform(boxToGlobal, voxCenterPoint, voxCenterPointT);

    // rotate translate
    voxCenterPoint[0] = R[0]*voxCenterPointT[0] + R[1]*voxCenterPointT[1] + R[2]*voxCenterPointT[2] + T[0];
    voxCenterPoint[1] = R[3]*voxCenterPointT[0] + R[4]*voxCenterPointT[1] + R[5]*voxCenterPointT[2] + T[1];
    voxCenterPoint[2] = R[6]*voxCenterPointT[0] + R[7]*voxCenterPointT[1] + R[8]*voxCenterPointT[2] + T[2];

    float voxCenterPointGNorm = sqrt(voxCenterPoint[0]*voxCenterPoint[0] + voxCenterPoint[1]*voxCenterPoint[1] + voxCenterPoint[2]*voxCenterPoint[2]);
    // normalize
    voxCenterPointT[0] = voxCenterPoint[0]/voxCenterPointGNorm;
    voxCenterPointT[1] = voxCenterPoint[1]/voxCenterPointGNorm;
    voxCenterPointT[2] = voxCenterPoint[2]/voxCenterPointGNorm;

    voxCenterPointT[2] += xi;
    voxCenterPointT[0] /= voxCenterPointT[2];
    voxCenterPointT[1] /= voxCenterPointT[2];
    voxCenterPointT[2] = 1;

    float voxCenterProj[3];

    voxCenterProj[0] = K[0]*voxCenterPointT[0] + K[1]*voxCenterPointT[1] + K[2]*voxCenterPointT[2];
    voxCenterProj[1] = K[3]*voxCenterPointT[0] + K[4]*voxCenterPointT[1] + K[5]*voxCenterPointT[2];

    // vox center point contains the 3d point in the local camera frame


    if (voxCenterPoint[2] >= minDepth && voxCenterPoint[2] <= maxDepth)
    {

        // get integer coordinates
        int xp,yp;
        xp = round(voxCenterProj[0]);
        yp = round(voxCenterProj[1]);

        // test if inside image
        if (xp >= 0 && xp < cols && yp >= 0 && yp < rows) {

            float depth = depthDataD[yp*cols + xp];

            if (depth > 0)
            {
                float dist = voxCenterPoint[2] - depth;

                if (fabs(dist) > delta1)
                {
                    const int voxInd = z*xRes*yRes + y*xRes + x;
                    if (fabs(dist) < delta2)
                    {
                        float weight = ((fabs(dist) - delta1)/(delta2 - delta1))*epsilon;
                        if (dist < 0)
                        {
                            // in front of surface
                            voxelD[voxInd] -= weight;
                        }
                        else if (dist > 0)
                        {
                            // behind surface
                            voxelD[voxInd] += weight;
                        }

                    }
                    else if(fabs(dist) < delta3)
                    {

                        if (dist < 0)
                        {
                            // in front of surface
                            voxelD[voxInd] -= epsilon;
                        }
                        else if (dist > 0)
                        {
                            // behind surface
                            voxelD[voxInd] += epsilon;
                        }
                    }
                    else
                    {
                        if (dist < 0)
                        {
                            voxelD[voxInd] -= eta*epsilon;
                        }
                    }

                }
            }
        }
    }
}

void integrateDepthMapWithFixedWeights(float* voxelD, float* boxToGlobalD,
                                       int xRes, int yRes, int zRes,
                                       float xBegin, float yBegin, float zBegin,
                                       float deltaX, float deltaY, float deltaZ,
                                       float* depthDataD, int rows, int cols,
                                       float maxDepth, float minDepth,
                                       float epsilon, float eta, float delta1, float delta2, float delta3,
                                       float* rMatD, float* tVecD, float* kMatD, float xi)
{
    dim3 dimGrid(xRes, yRes);
    dim3 dimBlock(zRes);

    integrationKernelWithFixedWeights<<<dimGrid,dimBlock>>>(voxelD, boxToGlobalD,
                                            xRes, yRes, zRes,
                                            xBegin, yBegin, zBegin,
                                            deltaX, deltaY, deltaZ,
                                            depthDataD, rows, cols,
                                            maxDepth, minDepth,
                                            epsilon, eta, delta1, delta2, delta3,
                                            rMatD, tVecD, kMatD, xi);

    D3D_CUDA_CHECK_ERROR

}

}
}

