#ifndef V3DRECONSTRUCTIONLOADER_H
#define V3DRECONSTRUCTIONLOADER_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <d3d_base/cameraMatrix.h>
#include <opencv2/core/core.hpp>

using std::string;
using std::vector;
using std::pair;
using Eigen::Matrix;

namespace D3D
{
    class V3DReconstructionLoader
    {
    public:
        V3DReconstructionLoader(string dirName);

        int getNumModels();
        int getNumCams(int modelNum);

        pair<int, CameraMatrix<double> > getCam(int modelNum, int camNum);
        cv::Mat loadImage(int imageNum);

    private:
        vector<string> images;
        vector<Eigen::Matrix<double, 3, 3> > Ks;
        int numModels;
        vector<vector<pair<int, CameraMatrix<double> > > > cams; // image num camera matrix pairs
        string dirName;

    };
}

#endif // V3DRECONSTRUCTIONLOADER_H
