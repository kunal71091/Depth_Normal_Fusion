#include <d3d_io/urbanScapeSequenceLoader.h>
#include <d3d_base/exception.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

using namespace D3D;

using cv::imread;

using boost::filesystem::path;
using boost::filesystem::exists;
using boost::filesystem::is_directory;
using boost::filesystem::directory_iterator;

using std::cerr;
using std::endl;
using std::ifstream;
using std::make_pair;

using Eigen::Matrix;

UrbanScapeSequenceLoader::UrbanScapeSequenceLoader(string dir)
{
    this->mainDirName = dir;
    curPos = 0;
    loadDirectory();
}

bool UrbanScapeSequenceLoader::atEOS()
{
    return (curPos >= cams.size());
}

void UrbanScapeSequenceLoader::moveNext()
{
    if (!atEOS())
    {
        curPos++;
    }
}

Mat UrbanScapeSequenceLoader::getImage()
{
//    cerr << images[cams[curPos].first] << endl;
//    cerr << cams[curPos].second << endl;
    return cv::imread(images[cams[curPos].first]);
}

CameraMatrix<double> UrbanScapeSequenceLoader::getCam()
{
    CameraMatrix<double> cam;

    if (!atEOS())
    {
        ifstream camFile;
        camFile.open(cams[curPos].second.c_str());

        Matrix<double, 3, 3> R;
        Matrix<double, 3, 1> C;

        string nextNumber;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                camFile >> nextNumber;
                double num = atof(nextNumber.c_str());

                R(i,j) = num;
            }
            camFile >> nextNumber;
            double num = atof(nextNumber.c_str());

            C(i) = num;
        }

        cam.setKRT(K, R, C);
    }

    return cam;
}

void UrbanScapeSequenceLoader::loadDirectory()
{
    if (!exists(mainDirName + "/K.txt") || !exists(mainDirName + "/undistorted/") ||
        !exists(mainDirName + "/tracker3d/"))
    {
        D3D_THROW_EXCEPTION( "not a valid data directory!" )
    }

    // load K matrix
    ifstream kFile;
    string kFileName = mainDirName + "/K.txt";
    kFile.open(kFileName.c_str());

    K.setIdentity();
    string nextNumber;
    for (int i = 0; i < 2; i++)
    {
        for (int j = i; j < 3; j++)
        {
            kFile >> nextNumber;
            double num = atof(nextNumber.c_str());

            K(i,j) = num;
        }
    }

    // load directory contents

    cams.clear();

    path camsPath(mainDirName + "/tracker3d/");

    if (is_directory(camsPath))
    {
        directory_iterator end_iter;
        for (directory_iterator dir_iter(camsPath); dir_iter != end_iter; ++dir_iter)
        {
            string filename = dir_iter->path().filename().string();
            if (filename.length() == 18)
            {
                if (filename.compare(0,6,"camera") == 0 && filename.compare(14,4,".txt") == 0)
                {
                    string numPart = filename.substr(6,8);
                    int num = atoi(numPart.c_str());
                    cams.push_back(make_pair(num, camsPath.string() + filename));
                }
            }
        }
    }

    sort(cams.begin(),cams.end());

    images.clear();

    path imgPath(mainDirName + "/undistorted/");

    if (is_directory(imgPath))
    {
       directory_iterator end_iter;
       for (directory_iterator dir_iter(imgPath); dir_iter != end_iter; ++dir_iter)
       {
           string filename = dir_iter->path().filename().string();
           if (filename.length() == 23)
           {
               if (filename.compare(0,11,"undistorted") == 0 && filename.compare(19,4,".ppm") == 0)
               {
                   string numPart = filename.substr(11,8);
                   int num = atoi(numPart.c_str());
                   images[num] = imgPath.string() + filename;
               }
           }
       }
    }







}
