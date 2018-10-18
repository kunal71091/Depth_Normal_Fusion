#include "fishEyeObstacleExtractor.h"
#include <d3d_base/grid.h>
#include <cmath>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "cudaFishEyeDepthIntegrator.h"

using namespace D3D;

FishEyeObstacleExtractor::FishEyeObstacleExtractor(int angularRes, int depthRes, float zNear, float zFar, float maxHeight,  float groundPlaneMargin, Eigen::Matrix3d cameraR, Eigen::Vector3d cameraC)
{
    this-> angularRes = angularRes;
    this->depthRes = depthRes;
    this->zNear = zNear;
    this->zFar = zFar;
    this->maxHeight = maxHeight;
    this->groundPlaneMargin = groundPlaneMargin;

    // compute the camera to grid transformation
    this->cameraC = cameraC;
    this->cameraR = cameraR;


    Eigen::Vector3d x_Cam;
    x_Cam(0) = 1;
    x_Cam(1) = x_Cam(2) = 0;
    x_Cam = cameraR*x_Cam;

    // for now the ground plane is aligned with positive z
    Eigen::Vector3d n;
    n(0) = 0;
    n(1) = 0;
    n(2) = 1;

    Eigen::Vector3d projection = x_Cam.dot(n)*(n);

    Eigen::Vector3d x_Grid = x_Cam - projection;

    x_Grid.normalize();

    Eigen::Vector3d z_Grid = x_Grid.cross(-n);

    R_Grid_to_Global.col(0) = x_Grid;
    R_Grid_to_Global.col(1) = -n;
    R_Grid_to_Global.col(2) = z_Grid;

    R_Camera_to_Grid = R_Grid_to_Global.transpose()*cameraR;
}

void FishEyeObstacleExtractor::processFishEyeDepthMap3D(FishEyeDepthMap<float, double> depthMap)
{
    D3D::Grid<float> weights;
    D3D::CudaFishEyeDepthIntegrator dI(800, 800, 50, 40, 40, 2.5, -20, -20, -0.5, 0.4, 500, Eigen::Matrix4f::Identity(), 1);

    FishEyeDepthMap<float, double> dMCopy = depthMap;
    D3D::FishEyeCameraMatrix<double> cam(depthMap.getCam().getK(), cameraR.transpose(), -cameraR.transpose()*cameraC, depthMap.getCam().getXi());
    dMCopy.setCam(cam);

    dI.setFixedWeights(1, 0, 0, 0, 0.25);

    dI.setTile(0);
    dI.addDepthMapWithFixedWeights(depthMap);

    weights = dI.downloadTile();

    for (int z = 0; z < 20; z++)
    {
        D3D::displayGridZSliceAsImage(weights, z, -1.0f, 1.0f, 10);
    }
}

void FishEyeObstacleExtractor::processFishEyeDepthMap(FishEyeDepthMap<float, double> depthMap)
{
    D3D::Grid<float> weights(angularRes, depthRes, 1, 0);

//    float dispStep = (1/zNear - 1/zFar)/(depthRes-1);
    float depthStep = (zFar-zNear)/(depthRes-1);

    D3D::FishEyeCameraMatrix<double> cam(depthMap.getCam().getK(), Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), depthMap.getCam().getXi());
    for (unsigned int y = 0; y < depthMap.getHeight(); y++)
        for (unsigned int x = 0; x < depthMap.getWidth(); x++)
        {
            if (depthMap(x,y) > 0)
            {
                Eigen::Vector4d point_cam_hom = cam.unprojectPoint(x,y,depthMap(x,y));
                if (point_cam_hom(3) == 1)
                {
                    Eigen::Vector3d point_cam = point_cam_hom.topRows(3);
                    Eigen::Vector3d point_grid = R_Camera_to_Grid*point_cam;
                    if (-point_grid(1) > maxHeight-cameraC(2))
                    {
                        depthMap(x,y) = -1;
                    }
                    else
                    {
                        float r = std::sqrt(point_grid(0)*point_grid(0) + point_grid(2)*point_grid(2));
                        float theta = atan2(point_grid(2), point_grid(0));

                        // find the right bins
                        int thetaPos = round(theta/M_PI*(angularRes-1));
                        int depthPos = round((r-zNear)/depthStep);
//                        int depthPos = round((1/r - 1/zFar)/dispStep);
                        if (depthPos >= 0 && depthPos < depthRes && thetaPos >= 0 && thetaPos < angularRes)
                        {

                            if(-point_grid(1) < groundPlaneMargin-cameraC(2))
                            {
                                depthMap(x,y) = -1;
                                weights(thetaPos, depthPos) -= 1;
                            }
                            else
                            {
                                weights(thetaPos, depthPos) += depthMap(x,y);
                                int i = 1;
                                while (i*depthStep < 0.25 && depthPos+i < depthRes)
                                {
                                    weights(thetaPos, depthPos+i) += depthMap(x,y);
                                    i++;
                                }
                            }
                        }
                    }
                }

            }
        }

    // find strongest obstacle
    D3D::Grid<float> obstacles(angularRes, depthRes, 1, 0);
    std::vector<float> obstacleDist(angularRes);
    for (unsigned int x = 0; x < obstacles.getWidth(); x++)
    {
        float sum = 0;
        float minEnergy = std::numeric_limits<float>::max();
        int obstaclePos = obstacles.getHeight();
        for (unsigned int y = 0; y < obstacles.getHeight()-2; y++)
        {
            sum += weights(x,y);
            if (sum-weights(x,y+1) <= minEnergy)
            {
                minEnergy = sum-weights(x,y+1);
                obstaclePos = y+1;
            }
        }
        if (minEnergy+weights(x,obstacles.getHeight()-1) < minEnergy)
        {
            minEnergy = minEnergy+weights(x,obstacles.getHeight()-1);
            obstaclePos = obstacles.getHeight();
        }

        float obstacleStrength = 0;

        int i = 0;
        while (i*depthStep < 0.5 && obstaclePos+i < depthRes)
        {
            obstacleStrength += weights(x, obstaclePos+i);
            i++;
        }

        if (weights(x,obstaclePos) < 50)
        {
            obstaclePos = obstacles.getHeight();
            obstacleDist[x] = -1;
        }
        else
        {
//            std::cout << obstaclePos << "    ";
//            obstacleDist[x] = 1.0/(obstaclePos*dispStep + 1/zFar);
            obstacleDist[x] = obstaclePos*depthStep + zNear;
//            std::cout << obstacleDist[x] << std::endl;
        }
//        std::cout << obstaclePos << std::endl;

        for (unsigned int y = 0; y < obstacles.getHeight(); y++)
        {
            if (obstaclePos > (int) y)
            {
                obstacles(x,y) = 1;
            }

        }
    }

    cv::Mat obstacleImg(400, 400, CV_8UC3);

    float xStep = 40.0/400;
    float yStep = 40.0/400;

    float xOrigin = 20;
    float yOrigin = 20;

    for (int y = 0; y < obstacleImg.rows; y++)
        for (int x = 0; x < obstacleImg.cols; x++)
        {
            // compute point in car frame
            Eigen::Vector3d carFramePoint;
            carFramePoint(0) = xStep*x - xOrigin;
            carFramePoint(1) = yStep*y - yOrigin;
            carFramePoint(2) = 0;

            // rotate the point to the grid frame;
            Eigen::Vector3d gridFramePoint;
            gridFramePoint = R_Grid_to_Global.transpose()*carFramePoint - R_Grid_to_Global.transpose()*cameraC;

            float xCoord = gridFramePoint(0);
            float yCoord = gridFramePoint(2);

            float theta = atan2(yCoord, xCoord);
            int thetaPos = round(theta/M_PI*(angularRes-1));
            if (thetaPos >= 0 && thetaPos < angularRes)
            {
                float currDist = std::sqrt(yCoord*yCoord + xCoord*xCoord);
                //            std::cout << currDist << std::endl;

                if(currDist < obstacleDist[thetaPos] || currDist - 0.5 > obstacleDist[thetaPos] || obstacleDist[thetaPos] == -1)
                {
                    obstacleImg.at<unsigned char>(y,3*x) = 0;
                    obstacleImg.at<unsigned char>(y,3*x+1) = 255;
                    obstacleImg.at<unsigned char>(y,3*x+2) = 0;
                }
                else
                {
                    obstacleImg.at<unsigned char>(y,3*x) = 0;
                    obstacleImg.at<unsigned char>(y,3*x+1) = 0;
                    obstacleImg.at<unsigned char>(y,3*x+2) = 255;
                }
            }
        }

    cv::imshow("obstacles", obstacleImg);
    cv::waitKey(1);

    // display
    D3D::displayGridZSliceAsImage(weights, 0, -1.0f, 1.0f, 50);




}
