#ifndef URBANSCAPESEQUENCELOADER_H
#define URBANSCAPESEQUENCELOADER_H

#include "monoImageSequenceLoader.h"

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

using std::string;
using std::vector;
using std::pair;
using std::map;


namespace D3D
{

    class UrbanScapeSequenceLoader : public MonoImageSequenceLoader
    {
    public:
        UrbanScapeSequenceLoader(string dir);
        bool atEOS();
        void moveNext();
        Mat getImage();
        CameraMatrix<double> getCam();
    private:
        string mainDirName;

        void loadDirectory();

        Eigen::Matrix<double, 3, 3> K;
        vector<pair<int, string> > cams;
        map<int, string> images;

        unsigned int curPos;
    };

}

#endif // URBANSCAPESEQUENCELOADER_H
