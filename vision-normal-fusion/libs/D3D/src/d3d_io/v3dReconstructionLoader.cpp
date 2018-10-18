#include "v3dReconstructionLoader.h"

#include <d3d_base/exception.h>

#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <iostream>
#include <sstream>


using namespace D3D;

using std::ifstream;
using std::cout;
using std::endl;
using std::stringstream;
using boost::filesystem::exists;

V3DReconstructionLoader::V3DReconstructionLoader(string dirName)
{
    this->dirName = dirName;

    // read image file names
    ifstream imageFile;
    imageFile.open((dirName + "/images.txt").c_str());
    if (!imageFile.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open images.txt");
    }
    images.clear();
    while(!imageFile.eof())
    {
        string imageFileName;
        imageFile >> imageFileName;
        if (imageFileName.size() > 0)
            images.push_back(imageFileName);
    }
    imageFile.close();

    // read calibration db
    ifstream calibrationFile;
    calibrationFile.open((dirName + "/calibration_db.txt").c_str());
    if (!calibrationFile.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open calibration_db.txt");
    }
    int nViews;
    calibrationFile >> nViews;
    Ks.resize(nViews);

    for (int i = 0; i < nViews; ++i)
    {
        Ks[i].setIdentity();

        calibrationFile >> Ks[i](0,0) >> Ks[i](0,1) >> Ks[i](0,2);
        calibrationFile >> Ks[i](1,1) >> Ks[i](1,2);

        double dist1, dist2, dist3, dist4;
        int dimX, dimY;

        calibrationFile >> dist1 >> dist2 >> dist3 >> dist4;
        calibrationFile >> dimX >> dimY;
    }
    calibrationFile.close();

    //load cameras
    cams.clear();
    int modelNum = 0;

    while (true)
    {
        // try to load next model
        stringstream modelFileName;
        modelFileName << dirName  << "/model-"  << modelNum << "-cams.txt";

        if (!exists(modelFileName.str()))
            break;

        // open file
        ifstream modelFile;
        modelFile.open(modelFileName.str().c_str());
        if (!modelFile.is_open())
        {
            D3D_THROW_EXCEPTION("Could not open model file" );
        }

        int numCams;
        modelFile >> numCams;


        vector<pair<int, CameraMatrix<double> > > model(numCams);


        for (int c = 0; c < numCams; c++)
        {
            modelFile >> model[c].first;

            Matrix<double, 3, 3> R;
            Matrix<double, 3, 1> T;

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                    modelFile >> R(i,j);

                modelFile >> T(i);
            }

            model[c].second.setKRT(Ks[model[c].first], R, T);

            // DEBUG: write out camera matrix
//            Matrix<double, 3, 3> Rb = model[c].second.getR();
//            for (int i = 0; i < 3; i++)
//            {
//                for (int j = 0; j < 3; j++)
//                    cout << Rb(i,j) << " ";
//                cout << endl;
//            }

//            Matrix<double, 3, 1> Tb = model[c].second.getT();
//            for (int i = 0; i < 3; i++)
//                cout << Tb(i,0) << endl;

        }

        cams.push_back(model);

        modelNum++;
    }
}

int V3DReconstructionLoader::getNumModels()
{
    return cams.size();
}

int V3DReconstructionLoader::getNumCams(int modelNum)
{
    if (!(modelNum >= 0 && modelNum < (int) cams.size()))
        D3D_THROW_EXCEPTION( "modelNum not valid" )

    return cams[modelNum].size();
}

pair<int, CameraMatrix<double> > V3DReconstructionLoader::getCam(int modelNum, int camNum)
{
    return cams[modelNum][camNum];
}

cv::Mat V3DReconstructionLoader::loadImage(int imageNum)
{
    return cv::imread((dirName + "/" + images[imageNum]).c_str());
}
