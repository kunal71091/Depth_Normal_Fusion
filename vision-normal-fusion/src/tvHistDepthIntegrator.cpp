#include <d3d_base/exception.h>
#include <d3d_base/configFile.h>

#include <d3d_base/grid.h>
#include <d3d_base/depthMap.h>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>

#include <Eigen/Dense>

struct Hist {
    float counts[8];
};

int main(int argc, char* argv[])
{
    std::string configFile;
    std::string depthMapListFile;
    std::string outputGridFile;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("configFile", boost::program_options::value<std::string>(&configFile)->default_value("conf.txt"), "Config file")
            ("depthMapListFile", boost::program_options::value<std::string>(&depthMapListFile)->default_value("depthMaps.txt"), "Depth map list file")
            ("projections", boost::program_options::value< std::vector<std::string> >()->multitoken(), "Projections P1, ..., PN: X' = PN*...*P1*X")
            ("outputGridFile", boost::program_options::value<std::string>(&outputGridFile)->default_value("intCSDFGrid.dat"), "Output data file with integrated depth data.")
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

    float hfClampFront = conf.getAsFloat("HF_CLAMP_FRONT");
    float hfClampBack = conf.getAsFloat("HF_CLAMP_BACK");
    float hfDelta = conf.getAsFloat("HF_DELTA");

    // now load the depth map file names
    std::ifstream depthMapsStream;
    depthMapsStream.open(depthMapListFile.c_str());

    if (!depthMapsStream.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open depth maps file list")
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

    D3D::Grid<Hist> volume(bpResX, bpResY, bpResZ);
    for (int x = 0; x < bpResX; ++x)
        for (int y = 0; y < bpResY; ++y)
            for (int z = 0; z < bpResZ; ++z) {
                for (int b = 0; b < 8; ++b) {
                    volume(x, y, z).counts[b] = 0;
                }
            }


    std::cout << "Integrating depth" << std::endl;

    float deltaX = bpSizeX/bpResX;
    float deltaY = bpSizeY/bpResY;
    float deltaZ = bpSizeZ/bpResZ;

    float xBegin = bpMinX + deltaX / 2.f;
    float yBegin = bpMinY + deltaY / 2.f;
    float zBegin = bpMinZ + deltaZ / 2.f;

    for (unsigned int d = 0; d < depthMapFileNames.size(); d++) {
        std::cout << d << " ";
        std::cout.flush();

        D3D::DepthMap<float, double> dM;
        dM.loadFromDataFile(depthMapFileNames[d]);

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
                                float dist = (projVoxelCenter(2) - depth) / hfDelta;

                                if (-hfClampFront < dist && dist < hfClampBack) {
                                    //if (dist <= -1.f) {
                                        // -inf
                                    //    volume(x, y, z).counts[0] += 1;
                                    //} else if (dist >= 1.f) {
                                    //    // +inf
                                    //    volume(x, y, z).counts[9] += 1;
                                //    } else
                                {
                                        // compute bin
                                        // -1 (0) | -5/7 (1) | -3/7 (2) | -1/7 (3) | 1/7 (4) | 3/7 (5) | 5/7 (6) | 1 (7)
//                                        int bin = int(7.f * (dist + 1.f) / 2.f + 1.5f);
//                                        volume(x, y, z).counts[bin] += 1;

//                                        // for debugging purposes
                                        int closestBin = 0;
                                        float closestDistance = std::numeric_limits<float>::max();
                                        for (int i = 0; i < 8; ++i) {
                                            float binCenter = 2.f*float(i)/7.f - 1.f;
                                            if (fabs(binCenter-dist) < closestDistance) {
                                                closestDistance = fabs(binCenter-dist);
                                                closestBin = i;
                                            }
                                        }
                                        volume(x, y, z).counts[closestBin] += 1;
//                                        if (bin != closestBin) {
//                                            std::cout << "wrong bin computation: dist: " << dist << ", is " << bin << ", should: " << closestBin  << std::endl;
//                                        }
//                                        // end debugging
                                    }
                                } else if (dist < 0) {
                                    int closestBin = 0;
                                    float closestDistance = std::numeric_limits<float>::max();
                                    for (int i = 0; i < 8; ++i) {
                                        float binCenter = 2.f*float(i)/7.f - 1.f;
                                        if (fabs(binCenter-dist) < closestDistance) {
                                            closestDistance = fabs(binCenter-dist);
                                            closestBin = i;
                                        }
                                    }
                                    volume(x, y, z).counts[closestBin] += 0.005;
                                }
                            }
                        } // end for z
                    } // end for y
                } // end for x
    } // end for d

    volume.saveAsDataFile(outputGridFile);
}
