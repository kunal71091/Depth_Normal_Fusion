#include <d3d_cudaBase/cudaCommon.h>
#include <iostream>

namespace D3D_CUDA
{
namespace CudaDepthIntegratorMultiClassDeviceCode
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
                             const float* projD, float centerX, float centerY, float centerZ, float beta, float skyWeight)
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

    float dir[3];
    dir[0] = voxCenterPointG[0]-centerX;
    dir[1] = voxCenterPointG[1]-centerY;
    dir[2] = voxCenterPointG[2]-centerZ;

    float maxComp = max(abs(dir[0]), max(abs(dir[1]), abs(dir[2])));

    dir[0] = dir[0]/maxComp;
    dir[1] = dir[1]/maxComp;
    dir[2] = dir[2]/maxComp;

    float directionWeight = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);


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

            // debug
//            int voxInd = (z*xRes*yRes + y*xRes + x)*numClasses;
//            voxelD[voxInd + freeSpaceClass] += 1;
//            voxelD[voxInd] += 2;

// all transition approach

            int voxInd = (z*xRes*yRes + y*xRes + x)*numClasses;

//            float bestScore = classScoresDataD[depthMapIdx*numClasses];
//            int bestClass = 0;

//            for (int i = 1; i < numClasses; i++)
//            {
//                if (classScoresDataD[depthMapIdx*numClasses + i] > bestScore)
//                {
//                    bestScore = classScoresDataD[depthMapIdx*numClasses + i];
//                    bestClass = i;
//                }
//            }

//            if (bestClass < 3)
//            {
  
  
	    int bestLabel = -1;
	    //int worstLabel = -1;
	    float bestScore = -1e6;
	    float secondBestScore = -1e6;
	    //float worstScore = 1e6;
	    float averageSolidScore = 0;
	    
	    for (int c = 0; c < numClasses; c++)
	    {
	      if (classScoresDataD[depthMapIdx*numClasses + c] > bestScore)
	      {
		secondBestScore = bestScore;
		bestScore = classScoresDataD[depthMapIdx*numClasses + c];
		bestLabel = c;
		
	      }
	      else if (classScoresDataD[depthMapIdx*numClasses + c] > secondBestScore)
	      {
		secondBestScore = classScoresDataD[depthMapIdx*numClasses + c];
	      }
	      
	      //if (c != freeSpaceClass && classScoresDataD[depthMapIdx*numClasses + c] < worstScore)
	      //{
		//worstScore = classScoresDataD[depthMapIdx*numClasses + c];
		//worstLabel = c;
	      //}

	      if (c != freeSpaceClass)
              {
               averageSolidScore += classScoresDataD[depthMapIdx*numClasses + c]*1.0/(numClasses - 1);
              }
	    }
  
            if (depth > 0)
            {

                float dist = voxCenterProj[2] - depth;
//                float distToNextDisparityPlane = max(((depth*depth)*uncertFact), epsilon);
//                float distForClassWeight = max(((depth*depth)*uncertFact), epsilon*2.0f/3.0f);
                float distToNextDisparityPlane = epsilon;
                float distForClassWeight1 =  epsilon*1.0f/2.0f;
                float distForClassWeight2 =  epsilon;
                float distForClassWeight3 =  epsilon*1.0f/4.0f;


                if (dist < distToNextDisparityPlane)
                {

                    // inside band
                    if (fabs(dist) < distToNextDisparityPlane)
                    {
                        if (dist < 0)
                        {
                            const float weight = directionWeight*epsilon/distToNextDisparityPlane;

                            // in front of surface
                            for (int i = 0; i < numClasses; i++)
                            {
//                                if (i == 4)
//                                    continue;

                                if (i != freeSpaceClass)
                                {
                                    voxelD[voxInd + i] += beta*weight;
                                }
                            }

                        }
                        else
                        {
                            const float weight = directionWeight*epsilon/distToNextDisparityPlane;
                            // behind surface
                            for (int i = 0; i < numClasses; i++)
                            {

                                if (i != freeSpaceClass)
                                {
                                    voxelD[voxInd + i] -= beta*weight;
				}


                                    // check if it is the right voxel for dist1
                                    if (distForClassWeight1 < dist + voxelDiameter && dist < distForClassWeight1)
                                    {

                                        // check if any of the neighboring voxels would be in the range too,
                                        // if so this one is only used if it is further from the camera

                                        float voxCenterPointNextG[3];
                                        float voxCenterPointNext[3];
                                        float voxCenterNextProj[3];

                                        bool useThis = true;

                                        for (int wx = -1; wx <= 1; wx++)
                                            for (int wy = -1; wy <= 1; wy++)
                                                for (int wz = -1; wz <= 1; wz++)
                                                {
                                                    voxCenterPointNext[0] = xBegin + (x+wx)*deltaX;
                                                    voxCenterPointNext[1] = yBegin + (y+wy)*deltaY;
                                                    voxCenterPointNext[2] = zBegin + (z+wz)*deltaZ;


                                                    transform(boxToGlobal, voxCenterPointNext, voxCenterPointNextG);
                                                    project(proj, voxCenterPointNextG, voxCenterNextProj);

                                                    int xpN = round(voxCenterNextProj[0]/voxCenterNextProj[2]);
                                                    int ypN = round(voxCenterNextProj[1]/voxCenterNextProj[2]);

                                                    if (xpN == xp && ypN == yp)
                                                    {

                                                        float distN = voxCenterNextProj[2] - depth;

                                                        if (distN > dist && distForClassWeight1 < distN + voxelDiameter && distN < distForClassWeight1)
                                                        {
                                                            useThis = false;
                                                        }
                                                    }

                                                }
                                        if (useThis)
                                        {
					    if (i != freeSpaceClass)
					    {
                                            voxelD[voxInd+i] -= 0.0f*rho*(classScoresDataD[depthMapIdx*numClasses + i]);
					    }
					    else
					    {
					      voxelD[voxInd+i] += 0.0f*rho*averageSolidScore;
					    }
                                        }
                                    }

                                    // check if it is the right voxel for dist2
                                    if (distForClassWeight2 < dist + voxelDiameter && dist < distForClassWeight2)
                                    {

                                        // check if any of the neighboring voxels would be in the range too,
                                        // if so this one is only used if it is further from the camera

                                        float voxCenterPointNextG[3];
                                        float voxCenterPointNext[3];
                                        float voxCenterNextProj[3];

                                        bool useThis = true;

                                        for (int wx = -1; wx <= 1; wx++)
                                            for (int wy = -1; wy <= 1; wy++)
                                                for (int wz = -1; wz <= 1; wz++)
                                                {
                                                    voxCenterPointNext[0] = xBegin + (x+wx)*deltaX;
                                                    voxCenterPointNext[1] = yBegin + (y+wy)*deltaY;
                                                    voxCenterPointNext[2] = zBegin + (z+wz)*deltaZ;


                                                    transform(boxToGlobal, voxCenterPointNext, voxCenterPointNextG);
                                                    project(proj, voxCenterPointNextG, voxCenterNextProj);

                                                    int xpN = round(voxCenterNextProj[0]/voxCenterNextProj[2]);
                                                    int ypN = round(voxCenterNextProj[1]/voxCenterNextProj[2]);

                                                    if (xpN == xp && ypN == yp)
                                                    {

                                                        float distN = voxCenterNextProj[2] - depth;

                                                        if (distN > dist && distForClassWeight2 < distN + voxelDiameter && distN < distForClassWeight2)
                                                        {
                                                            useThis = false;
                                                        }
                                                    }

                                                }
                                        if (useThis)
                                        {
					  if (i != freeSpaceClass)
					  {
                                            voxelD[voxInd+i] -= 1.0f*rho*(classScoresDataD[depthMapIdx*numClasses + i]);
					   }
					   else
					   {
					     voxelD[voxInd+i] += 1.0f*rho*averageSolidScore;
					   }
                                        }
                                    }

                                    // check if it is the right voxel for dist3
                                    if (distForClassWeight3 < dist + voxelDiameter && dist < distForClassWeight3)
                                    {

                                        // check if any of the neighboring voxels would be in the range too,
                                        // if so this one is only used if it is further from the camera

                                        float voxCenterPointNextG[3];
                                        float voxCenterPointNext[3];
                                        float voxCenterNextProj[3];

                                        bool useThis = true;

                                        for (int wx = -1; wx <= 1; wx++)
                                            for (int wy = -1; wy <= 1; wy++)
                                                for (int wz = -1; wz <= 1; wz++)
                                                {
                                                    voxCenterPointNext[0] = xBegin + (x+wx)*deltaX;
                                                    voxCenterPointNext[1] = yBegin + (y+wy)*deltaY;
                                                    voxCenterPointNext[2] = zBegin + (z+wz)*deltaZ;


                                                    transform(boxToGlobal, voxCenterPointNext, voxCenterPointNextG);
                                                    project(proj, voxCenterPointNextG, voxCenterNextProj);

                                                    int xpN = round(voxCenterNextProj[0]/voxCenterNextProj[2]);
                                                    int ypN = round(voxCenterNextProj[1]/voxCenterNextProj[2]);

                                                    if (xpN == xp && ypN == yp)
                                                    {

                                                        float distN = voxCenterNextProj[2] - depth;

                                                        if (distN > dist && distForClassWeight3 < distN + voxelDiameter && distN < distForClassWeight3)
                                                        {
                                                            useThis = false;
                                                        }
                                                    }

                                                }

                                        if (useThis)
                                        {
					    if (i != freeSpaceClass)
					    {
                                              voxelD[voxInd+i] -= 0.0f*rho*(classScoresDataD[depthMapIdx*numClasses + i]);
					    }
					    else
					    {
					      voxelD[voxInd+i] += 0.0f*rho*averageSolidScore;
					    }
                                        }
                                    }
			    }
                        }
                        // viewing ray
                    }
                    else
                    {
                        float weight = directionWeight*eta*epsilon/distToNextDisparityPlane;
                        voxelD[voxInd+freeSpaceClass] -= beta*weight;
                    }
//                else
//                {
//                    const float weightInBand = epsilon/distToNextDisparityPlane;
//                    const float distBehindNextPlane = dist - distToNextDisparityPlane;
//                    const float weight = weightInBand*exp(-(distBehindNextPlane)/(epsilon*0.5));
//                    // far behind lets do an exponential decay to get this whole thing working
//                    for (int i = 0; i < numClasses; i++)
//                    {
//                        if (i != freeSpaceClass)
//                        {
//                            voxelD[voxInd + i] -= weight;
//                        }
//                    }
//                }
		}
            }


            // this is done even if we have a depth for the pixel
            {
                // enter weights for free space originating from sky
//                if (exp(classScoresDataD[depthMapIdx*numClasses + freeSpaceClass]) > 0.95)
//                {
                    if( bestLabel == freeSpaceClass)
                    {
                        // find second best class
//                         float secondBestScore = -1e6;
//                         for (int c = 0; c < numClasses; c++)
//                         {
//                             if (c != freeSpaceClass && classScoresDataD[depthMapIdx+numClasses + c] > secondBestScore)
//                             {
//                                 secondBestScore = classScoresDataD[depthMapIdx+numClasses + c];
//                             }
//                         }

                        voxelD[voxInd+freeSpaceClass] -= directionWeight*rho*skyWeight*(bestScore - secondBestScore);
                    }
//                    float sky_weight = max(0.0f, classScoresDataD[depthMapIdx*numClasses + freeSpaceClass]);

//                    voxelD[voxInd+freeSpaceClass] -= (50.0f)/((xRes + yRes + zRes)) * directionWeight * rho *sky_weight;
//                }
            }
//            }

//// only free space / occupied space transition approach

//            if (depth > 0)
//            {ls c

//                float dist = voxCenterProj[2] - depth;
//                float distToNextDisparityPlane = max(((depth*depth)*uncertFact), epsilon);

//                int voxInd = (z*xRes*yRes + y*xRes + x)*numClasses;

//                if (dist < distToNextDisparityPlane)
//                {

//                    // inside band
//                    if (fabs(dist) < distToNextDisparityPlane)
//                    {
//                        if (dist < 0)
//                        {
//                            const float weight = epsilon/distToNextDisparityPlane;
//                            voxelD[voxInd + freeSpaceClass] -= weight;

////                            for (int i = 0; i < numClasses; i++)
////                            {
////                                if (i != freeSpaceClass)
////                                {
////                                    voxelD[voxInd + i] += rho*(classScoresDataD[depthMapIdx*numClasses + freeSpaceClass] - classScoresDataD[depthMapIdx*numClasses + i]);
////                                }
////                            }

//                        }
//                        else
//                        {
//                            // behind surface
//                            const float weight = epsilon/distToNextDisparityPlane;
//                            voxelD[voxInd + freeSpaceClass] += weight;

//                            for (int i = 0; i < numClasses; i++)
//                            {
//                                if (i != freeSpaceClass)
//                                {
//                                    voxelD[voxInd + i] += rho*(classScoresDataD[depthMapIdx*numClasses + freeSpaceClass] - classScoresDataD[depthMapIdx*numClasses + i]);
//                                }
//                            }
//                        }
//                        // viewing ray
//                    }
//                    else
//                    {
//                        float weight = eta*epsilon/distToNextDisparityPlane;
//                        voxelD[voxInd+freeSpaceClass] -= weight;

////                        for (int i = 0; i < numClasses; i++)
////                        {
////                            if (i != freeSpaceClass)
////                            {
////                                voxelD[voxInd + i] += rho*(classScoresDataD[depthMapIdx*numClasses + freeSpaceClass] - classScoresDataD[depthMapIdx*numClasses + i]);
////                            }
////                        }
//                    }

//                }
//            }
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
                       const float* projD, float centerX, float centerY, float centerZ, float beta, float skyWeight)
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
                                            projD, centerX, centerY, centerZ, beta, skyWeight);
    D3D_CUDA_CHECK_ERROR

}

}
}

