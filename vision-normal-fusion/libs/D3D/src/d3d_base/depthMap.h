#ifndef DEPTHMAP_H
#define DEPTHMAP_H

#include "cameraMatrix.h"
#include <boost/shared_array.hpp>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace D3D
{

    using boost::shared_array;
    using std::ofstream;
    using std::string;

    template<typename T, typename U>
    class DepthMap
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DepthMap();
        DepthMap(unsigned int width, unsigned int height, const CameraMatrix<U>& cam);
        void initialize(T value);
        DepthMap& operator*= (T scale);
        T* getDataPtr();
        unsigned int getWidth() const;
        unsigned int getHeight() const;
        T& operator() (int x, int y);
        const T& operator() (int x, int y) const;
        const CameraMatrix<U>& getCam() const;
        void setCam(const CameraMatrix<U>& new_cam);

        DepthMap<T, U> clone() const;
        T* getDataPtr() const;

        void display(T minZ, T maxZ, int displayTime = 0, const char* windowName = "depth map");
        void displayInvDepth(T minZ, T maxZ, int displayTime = 0, const char* windowName = "inverse depth map");
        void displayInvDepthColored(T minZ, T maxZ, int displayTime = 0, const char* windowName = "color coded inverse depth map");

        // returns a point with homogeneous component 0 if depth is invalid
        Eigen::Matrix<U, 4, 1> unproject(int x, int y) const;

        void meshToVRML(ofstream& os, string textureImageFileName, float scale, float maxDispDiff, U maxDist = -1);
        void meshToVRML(ofstream& os, U maxDist = -1, T maxDispDiff = -1);
        void pointCloudToVRML(ofstream& os, U maxDist = -1);
        void pointCloudColoredToVRML(ofstream& os, cv::Mat image, U maxDist = -1);
        void saveAsImage(string fileName, T minDepth, T maxDepth);
        void saveInvDepthAsImage(string fileName, T minDepth, T maxDepth);
        void saveInvDepthAsColorImage(string fileName, T minZ, T maxZ);

        void saveAsDataFile(string fileName);
        void loadFromDataFile(string fileName);
        void setRT(const Eigen::Matrix<U, 3, 4>& RT);

    private:       
        shared_array<T> depths;
        unsigned int width;
        unsigned int height;
        CameraMatrix<U> cam;
    };
}

#endif // DEPTHMAP_H
