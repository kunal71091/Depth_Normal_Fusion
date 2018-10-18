#include <d3d_cudaBase/cudaCommon.h>
#include <iostream>

namespace D3D_CUDA
{
namespace CudaDepthIntegratorDeviceCode
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

__global__ void integrationKernelWithFixedWeights(float* voxelD, float* boxToGlobalD,
                                                  int xRes, int yRes, int zRes,
                                                  float xBegin, float yBegin, float zBegin,
                                                  float deltaX, float deltaY, float deltaZ,
                                                  float* depthDataD, int rows, int cols,
                                                  float maxDepth, float minDepth,
                                                  float epsilon, float eta, float delta1, float delta2, float delta3,
                                                  float* projD)
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
        if (xp >= 0 && xp < cols && yp >= 0 && yp < rows) {

            float depth = depthDataD[yp*cols + xp];

            if (depth > 0)
            {
                float dist = voxCenterProj[2] - depth;

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


void initTileDebug(int xRes, int yRes, int zRes, float** voxelsD)
{
    const int numBytes = xRes*yRes*zRes*sizeof(float);
    *voxelsD = new float[xRes*yRes*zRes];
    memset(*voxelsD, 0, numBytes);
}

void releaseDeviceMemoryDebug(void *addr)
{
    delete [] addr;
}

void allocateAndUploadTransformationDebug(float* transformationH, float** transformationD)
{
    *transformationD = new float[4*4];
    memcpy(*transformationD, transformationH, 4*4*sizeof(float));
}

void allocateAndUploadProjectionDebug(float* projectionH, float** projectionD)
{
    *projectionD = new float[3*4];
    memcpy(*projectionD, projectionH, 3*4*sizeof(float));
}

void allocateAndUploadDepthDataDebug(float* depthDataH, int rows, int cols, float** depthDataD)
{
    *depthDataD = new float[rows*cols];
    memcpy(*depthDataD, depthDataH, rows*cols*sizeof(float));
}

void downloadTileDebug(int xRes, int yRes, int zRes, float* voxelsD, float* voxelsH)
{
    const int numBytes = xRes*yRes*zRes*sizeof(float);
    memcpy(voxelsH, voxelsD, numBytes);
}

void projectDebug(float* projMat, float* point, float* projPoint)
{
    projPoint[0] = projMat[0]*point[0] + projMat[1]*point[1] + projMat[2]*point[2] + projMat[3];
    projPoint[1] = projMat[4]*point[0] + projMat[5]*point[1] + projMat[6]*point[2] + projMat[7];
    projPoint[2] = projMat[8]*point[0] + projMat[9]*point[1] + projMat[10]*point[2] + projMat[11];
}

void transformDebug(float* transformMat, float* point, float* transfPoint)
{
    transfPoint[0] = transformMat[0]*point[0] + transformMat[1]*point[1] + transformMat[2]*point[2] + transformMat[3];
    transfPoint[1] = transformMat[4]*point[0] + transformMat[5]*point[1] + transformMat[6]*point[2] + transformMat[7];
    transfPoint[2] = transformMat[8]*point[0] + transformMat[9]*point[1] + transformMat[10]*point[2] + transformMat[11];
}


void integrationKernelWithFixedWeightsDebug(float* voxelD, float* boxToGlobalD,
                                                  int xRes, int yRes, int zRes,
                                                  float xBegin, float yBegin, float zBegin,
                                                  float deltaX, float deltaY, float deltaZ,
                                                  float* depthDataD, int rows, int cols,
                                                  float maxDepth, float minDepth,
                                                  float epsilon, float eta, float delta1, float delta2, float delta3,
                                                  float* projD)
{
    for (int x = 0; x < xRes; ++x) {
        for (int y = 0; y < yRes; ++y) {
            for (int z = 0; z < zRes; ++z) {
                // voxel center in box coordinate frame
                float voxCenterPoint[3];
                voxCenterPoint[0] = xBegin + x*deltaX;
                voxCenterPoint[1] = yBegin + y*deltaY;
                voxCenterPoint[2] = zBegin + z*deltaZ;

                float voxCenterPointG[3];

                transformDebug(boxToGlobalD, voxCenterPoint, voxCenterPointG);
                //std::cout << voxCenterPoint[0] << "; " << voxCenterPoint[1] << "; " << voxCenterPoint[2] << std::endl;
                //std::cout << voxCenterPointG[0] << "; " << voxCenterPointG[1] << "; " << voxCenterPointG[2] << std::endl;

                float voxCenterProj[3];
                projectDebug(projD, voxCenterPointG, voxCenterProj);

                //std::cout << voxCenterProj[0] << "; " << voxCenterProj[1] << "; " << voxCenterProj[2] << std::endl << std::endl;

                if (voxCenterProj[2] >= minDepth && voxCenterProj[2] <= maxDepth)
                {
                    // perspective division
                    int xp,yp;
                    xp = round(voxCenterProj[0]/voxCenterProj[2]);
                    yp = round(voxCenterProj[1]/voxCenterProj[2]);

                    // test if inside image
                    if (xp >= 0 && xp < cols && yp >= 0 && yp < rows) {

                        float depth = depthDataD[yp*cols + xp];

                        if (depth > 0)
                        {
                            float dist = voxCenterProj[2] - depth;

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
                                       float* projD)
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
                                            projD);

    D3D_CUDA_CHECK_ERROR

//    integrationKernelWithFixedWeightsDebug(voxelD, boxToGlobalD,
//                                            xRes, yRes, zRes,
//                                            xBegin, yBegin, zBegin,
//                                            deltaX, deltaY, deltaZ,
//                                            depthDataD, rows, cols,
//                                            maxDepth, minDepth,
//                                            epsilon, eta, delta1, delta2, delta3,
//                                            projD);

}

}
}

