#ifndef FISHEYEDPETHMAP_H
#define FISHEYEDPETHMAP_H

#include "fishEyeCameraMatrix.h"
#include <string>

#include <fstream>

#include <opencv2/core/core.hpp>

#include <boost/shared_array.hpp>

namespace D3D
{
    template<typename T, typename U>
    class FishEyeDepthMap
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FishEyeDepthMap();
        FishEyeDepthMap(unsigned int width, unsigned int height, const FishEyeCameraMatrix<U>& cam);

        T& operator() (int x, int y);
        const T& operator() (int x, int y) const;

        T* getDataPtr();

        unsigned int getWidth() const;
        unsigned int getHeight() const;
        const FishEyeCameraMatrix<U>& getCam() const;
        void setCam(const FishEyeCameraMatrix<U>& cam);

        void display(T minZ, T maxZ, int displayTime = 0, const char* windowName = "depth map");
        void displayColored(T minZ, T maxZ, int displayTime = 0, const char* windowName = "depth map colored");
        void displayInvDepth(T minZ, T maxZ, int displayTime = 0, const char* windowName = "inverse depth map");
        void displayInvDepthColored(T minZ, T maxZ, int displayTime = 0, const char* windowName = "color coded inverse depth map");

        void saveInvDepthAsColorImage(std::string fileName, T minZ, T maxZ);

        // returns a point with homogeneous component 0 if depth is invalid
        Eigen::Matrix<U, 4, 1> unproject(int x, int y) const;

        void meshToVRML(std::ofstream& os, std::string textureImageFileName, float scale, float maxDispDiff, U maxDist = -1);
        void pointCloudToVRML(std::ofstream& os, U maxDist = -1);
        void pointCloudColoredToVRML(std::ofstream& os, cv::Mat image, U maxDist = -1);

        void saveAsDataFile(std::string fileName);
        void loadFromDataFile(std::string fileName);
    private:
        boost::shared_array<T> depths;
        unsigned int width;
        unsigned int height;
        FishEyeCameraMatrix<U> cam;

    };
}

#endif // FISHEYEDEPTHMAP_H
