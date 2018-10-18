#include <d3d_base/exception.h>
#include <d3d_base/configFile.h>

#include <d3d_base/grid.h>
#include <d3d_base/depthMap.h>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>

#include <Eigen/Dense>

//#include <d3d_base/exception.h>

//#include <boost/program_options.hpp>

//#include <fstream>
//#include <iostream>
//#include <d3d_base/configFile.h>
//#include <d3d_fusion/cudaDepthIntegrator.h>
//#include <d3d_fusion/cudaFaceDepthLabelIntegrator.h>
//#include <d3d_fusion/volumetricFusionTools.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <boost/filesystem.hpp>
//#include <iomanip>
//#include <opencv2/highgui/highgui.hpp>
//#include <d3d_fusion/cudaVisibleSpaceDetector.h>


//#include <boost/lexical_cast.hpp>
//#include <boost/algorithm/string.hpp>
//#include <boost/random.hpp>

//#include <Eigen/Dense>

struct Hist {
    unsigned char counts[10];
};

int main(int argc, char* argv[])
{
    std::string configFile;
    std::string depthMapListFile;
    std::string normalsListFile;
    std::string outputFolder;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("configFile", boost::program_options::value<std::string>(&configFile)->default_value("conf.txt"), "Config file")
            ("depthMapListFile", boost::program_options::value<std::string>(&depthMapListFile)->default_value("depthMaps.txt"), "Depth map list file")
            ("normalMapListFile", boost::program_options::value<std::string>(&normalsListFile)->default_value("normals.txt"), "Normals list file")
            ("projections", boost::program_options::value< std::vector<std::string> >()->multitoken(), "Projections P1, ..., PN: X' = PN*...*P1*X")
            ("outputFolder", boost::program_options::value<std::string>(&outputFolder)->default_value("integratedNormals"), "Output folder for integrated normals.")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    // config file
    D3D::ConfigFile conf(configFile);

    if (!conf.isFileRead())
    {
        std::cout << desc << std::endl;
        D3D_THROW_EXCEPTION("Could not open config file.")
    }

    // load the box parameters
    std::string bpMinXStr = conf.get("BP_MIN_X");
    if (bpMinXStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_MIN_X not specified in the config file.")
    }
    float bpMinX = atof(bpMinXStr.c_str());

    std::string bpSizeXStr = conf.get("BP_SIZE_X");
    if (bpSizeXStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_SIZE_X not specified in the config file.")
    }
    float bpSizeX = atof(bpSizeXStr.c_str());

    std::string bpMinYStr = conf.get("BP_MIN_Y");
    if (bpMinYStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_MIN_Y not specified in the config file.")
    }
    float bpMinY = atof(bpMinYStr.c_str());

    std::string bpSizeYStr = conf.get("BP_SIZE_Y");
    if (bpSizeYStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_SIZE_Y not specified in the config file.")
    }
    float bpSizeY = atof(bpSizeYStr.c_str());

    std::string bpMinZStr = conf.get("BP_MIN_Z");
    if (bpMinZStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_MIN_Z not specified in the config file.")
    }
    float bpMinZ = atof(bpMinZStr.c_str());

    std::string bpSizeZStr = conf.get("BP_SIZE_Z");
    if (bpSizeZStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_SIZE_Z not specified in the config file.")
    }
    float bpSizeZ = atof(bpSizeZStr.c_str());

    std::string bpResXStr = conf.get("BP_RES_X");
    if (bpResXStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_RES_X not specified in the config file.")
    }
    int bpResX = atoi(bpResXStr.c_str());

    std::string bpResYStr = conf.get("BP_RES_Y");
    if (bpResYStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_RES_Y not specified in the config file.")
    }
    int bpResY = atoi(bpResYStr.c_str());

    std::string bpResZStr = conf.get("BP_RES_Z");
    if (bpResZStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter BP_RES_Z not specified in the config file.")
    }
    int bpResZ = atoi(bpResZStr.c_str());

    std::string dmiMinDepthStr = conf.get("DMI_MIN_DEPTH");
    if (dmiMinDepthStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter DMI_MIN_DEPTH not specified in the config file.")
    }
    float dmiMinDepth = atof(dmiMinDepthStr.c_str());

    std::string dmiMaxDepthStr = conf.get("DMI_MAX_DEPTH");
    if (dmiMaxDepthStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter DMI_MAX_DEPTH not specified in the config file.")
    }
    float dmiMaxDepth = atof(dmiMaxDepthStr.c_str());

    // now load the depth map file names
    std::ifstream depthMapsStream;
    depthMapsStream.open(depthMapListFile.c_str());
    if (!depthMapsStream.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open depth maps file list")
    }

    std::ifstream normalsStream;
    normalsStream.open(normalsListFile.c_str());
    if (!normalsStream.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open normals file list")
    }

    std::string bpOriginStr = conf.get("BP_ORIGIN");
    Eigen::Vector3f bpOrigin;
    if (bpOriginStr.empty())
    {
        bpOrigin = Eigen::Vector3f::Zero();
    }
    else
    {
        std::stringstream bpOriginStream(bpOriginStr);
        bpOriginStream >> bpOrigin(0);
        bpOriginStream >> bpOrigin(1);
        bpOriginStream >> bpOrigin(2);

        if (bpOriginStream.fail())
        {
            D3D_THROW_EXCEPTION("Error reading BP_ORIGIN");
        }
    }

    std::string bpXAxisStr = conf.get("BP_X_AXIS");
    Eigen::Vector3f bpXAxis;
    bool xAxisSet = false;
    if (!bpXAxisStr.empty())
    {
        xAxisSet = true;

        std::stringstream bpXAxisStream(bpXAxisStr);
        bpXAxisStream >> bpXAxis(0);
        bpXAxisStream >> bpXAxis(1);
        bpXAxisStream >> bpXAxis(2);

        if (bpXAxisStream.fail())
        {
            D3D_THROW_EXCEPTION("Error reading BP_X_AXIS");
        }
    }

    std::string bpYAxisStr = conf.get("BP_Y_AXIS");
    Eigen::Vector3f bpYAxis;
    bool yAxisSet = false;
    if (!bpYAxisStr.empty())
    {
        yAxisSet = true;

        std::stringstream bpYAxisStream(bpYAxisStr);
        bpYAxisStream >> bpYAxis(0);
        bpYAxisStream >> bpYAxis(1);
        bpYAxisStream >> bpYAxis(2);

        if (bpYAxisStream.fail())
        {
            D3D_THROW_EXCEPTION("Error reading BP_Y_AXIS");
        }
    }

    std::string bpZAxisStr = conf.get("BP_Z_AXIS");
    Eigen::Vector3f bpZAxis;
    bool zAxisSet = false;
    if (!bpZAxisStr.empty())
    {
        zAxisSet = true;

        std::stringstream bpZAxisStream(bpZAxisStr);
        bpZAxisStream >> bpZAxis(0);
        bpZAxisStream >> bpZAxis(1);
        bpZAxisStream >> bpZAxis(2);

        if (bpZAxisStream.fail())
        {
            D3D_THROW_EXCEPTION("Error reading BP_Z_AXIS");
        }
    }

    if (xAxisSet == false && yAxisSet == false && zAxisSet == false)
    {
        bpXAxis(0) = 1; bpXAxis(1) = 0; bpXAxis(2) = 0;
        bpYAxis(0) = 0; bpYAxis(1) = 1; bpYAxis(2) = 0;
        bpZAxis(0) = 0; bpZAxis(1) = 0; bpZAxis(2) = 1;
    }
    else if (!(xAxisSet && yAxisSet && zAxisSet))
    {
        D3D_THROW_EXCEPTION("Either all or none out of BP_X_AXIS, BP_Y_AXIS and BP_Z_AXIS have to be specified.")
    }

    // rectify the axes
    // normalize
    bpXAxis /= bpXAxis.norm();
    bpYAxis /= bpYAxis.norm();
    bpZAxis /= bpZAxis.norm();

    Eigen::Vector3f rightAngleZAxis = bpXAxis.cross(bpYAxis);
    if (rightAngleZAxis.dot(bpZAxis) < 0.95)
    {
        D3D_THROW_EXCEPTION("The given axes are not orthogonal to each other.")
    }

    bpZAxis = rightAngleZAxis;

    Eigen::Vector3f rightAngleYAxis = bpZAxis.cross(bpXAxis);
    if (rightAngleYAxis.dot(bpYAxis) < 0.95)
    {
        D3D_THROW_EXCEPTION("The given axes are not orthogonal to each other.");
    }

    bpYAxis = rightAngleYAxis;

    Eigen::Matrix4f boxToGlobal = Eigen::Matrix4f::Identity();
    boxToGlobal.topLeftCorner(3,3).col(0) = bpXAxis;
    boxToGlobal.topLeftCorner(3,3).col(1) = bpYAxis;
    boxToGlobal.topLeftCorner(3,3).col(2) = bpZAxis;
    boxToGlobal.topRightCorner(3,1).col(0) = bpOrigin;

    // Load projections
    Eigen::Matrix4f allProj = Eigen::Matrix4f::Identity();
    if (!vm["projections"].empty()) {
        std::vector<std::string> projections = vm["projections"].as<std::vector<std::string> >();
        for (unsigned int p = 0; p < projections.size(); ++p) {
            std::cout << "loading projection " << projections[p] << std::endl;
            std::ifstream projectionFile;
            projectionFile.open(projections[p].c_str());
            // Parse matrix
            Eigen::Matrix4f proj;
            std::string tmp;
            int row = 0;
            if (!projectionFile) {
                std::string message = "Projection " + projections[p] + " does not exist!";
                D3D_THROW_EXCEPTION(message.c_str());
            } else {
                while (std::getline(projectionFile, tmp) && row < 4) {
                    boost::trim_if(tmp, boost::is_any_of("[ ]|"));
                    std::vector<std::string> tokens;
                    boost::split(tokens, tmp, boost::is_any_of("\t "), boost::token_compress_on);
                    for (unsigned int j = 0; j < tokens.size(); j++) {
                        proj(row, j) = boost::lexical_cast<float>(tokens[j]);
                    }
                    row++;
                }
            }
            allProj = proj*allProj;
        }
    } else {
        std::cout << "Warning! Did not load any projections!" << std::endl;
    }

    boxToGlobal = allProj.inverse()*boxToGlobal;

    std::vector<std::string> depthMapFileNames;
    std::string depthMapFileName;
    while (depthMapsStream >> depthMapFileName)
    {
        depthMapFileNames.push_back(depthMapFileName);
    }

    std::vector<std::string> normalFileNames;
    std::string normalFileName;
    while (normalsStream >> normalFileName)
    {
        normalFileNames.push_back(normalFileName);
    }

    std::cout << "Integrating normals" << std::endl;

    float deltaX = bpSizeX/bpResX;
    float deltaY = bpSizeY/bpResY;
    float deltaZ = bpSizeZ/bpResZ;

    float xBegin = bpMinX + deltaX / 2.f;
    float yBegin = bpMinY + deltaY / 2.f;
    float zBegin = bpMinZ + deltaZ / 2.f;

    boost::filesystem::path dir(outputFolder);
    if(boost::filesystem::create_directory(dir))
    {
        std::cout << "Directory Created: "<< outputFolder <<std::endl;
    }


    for (unsigned int d = 0; d < depthMapFileNames.size(); d++) {
        D3D::Grid<Eigen::Vector3f> volume(bpResX, bpResY, bpResZ);
        for (int x = 0; x < bpResX; ++x)
            for (int y = 0; y < bpResY; ++y)
                for (int z = 0; z < bpResZ; ++z) {
                    volume(x, y, z) = Eigen::Vector3f(0.f, 0.f, 0.f);
                }

        std::cout << d << " ";
        std::cout.flush();

        D3D::DepthMap<float, double> dM;
        dM.loadFromDataFile(depthMapFileNames[d]);

        D3D::Grid<float> normalGrid;
        normalGrid.loadFromDataFile(normalFileNames[d]);

//        cv::Mat nM = cv::imread(normalMapFileNames[d]);

        for (int x = 0; x < bpResX; ++x)
            for (int y = 0; y < bpResY; ++y)
                for (int z = 0; z < bpResZ; ++z) {
                    // voxel center in box coordinate frame
                    Eigen::Vector4f voxelCenter;
                    voxelCenter << xBegin + x*deltaX, yBegin + y*deltaY, zBegin + z*deltaZ, 1.f;

                    Eigen::Vector3f projVoxelCenter = dM.getCam().getP().cast<float>()*boxToGlobal*voxelCenter;

                    if (projVoxelCenter(2) >= dmiMinDepth && projVoxelCenter(2) <= dmiMaxDepth) {
                        // perspective division
                        int xp, yp;

                        xp = round(projVoxelCenter(0) / projVoxelCenter(2));
                        yp = round(projVoxelCenter(1) / projVoxelCenter(2));

                        // test if inside image
                        if (xp >= 0 && unsigned(xp) < dM.getWidth() && yp >= 0 && unsigned(yp) < dM.getHeight()) {
                            float depth = dM(xp, yp);
                            if (depth > 0.f) {
                                float dist = projVoxelCenter(2) - depth;

                                if (fabs(dist) < sqrt(0.25*(deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ))) {
                                    Eigen::Vector3f normal;
                                    normal << normalGrid(xp, yp, 0), normalGrid(xp, yp, 1), normalGrid(xp, yp, 2);

                                    if (fabs(normal.norm()-1.f) < 0.01) {
                                        volume(x, y, z) = boxToGlobal.topLeftCorner(3,3).transpose().inverse() * normal.normalized();
                                    }
                                }
                            }
                        } // end for z
                    } // end for y
                } // end for x
        std::stringstream inss;
        inss << outputFolder << "/normals_";
        inss.width(4);
        inss.fill('0');
        inss << d << ".dat";

        volume.saveAsDataFile(inss.str());
    } // end for d
}
