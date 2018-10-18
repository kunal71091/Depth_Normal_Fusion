#ifndef FISHEYEOBSTACLEEXTRACTOR_H
#define FISHEYEOBSTACLEEXTRACTOR_H

#include <d3d_base/fishEyeDepthMap.h>

#include <Eigen/Dense>

namespace D3D
{

class FishEyeObstacleExtractor
{
public:
    FishEyeObstacleExtractor(int angularRes, int depthRes, float zNear, float zFar, float maxHeight, float groundPlaneMargin, Eigen::Matrix3d cameraR, Eigen::Vector3d cameraC);

    void processFishEyeDepthMap(D3D::FishEyeDepthMap<float, double> depthMap);
    void processFishEyeDepthMap3D(FishEyeDepthMap<float, double> depthMap);

private:
    int angularRes;
    int depthRes;
    float zNear;
    float zFar;
    float maxHeight;
    float groundPlaneMargin;
    Eigen::Matrix3d cameraR;
    Eigen::Vector3d cameraC;

    Eigen::Matrix3d R_Camera_to_Grid;
    Eigen::Matrix3d R_Grid_to_Global;
};

}

#endif //FISHEYEOBSTACLEEXTRACTOR_H
