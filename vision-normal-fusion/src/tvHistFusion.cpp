#include <boost/program_options.hpp>
#include <string>
#include <d3d_base/grid.h>
#include <iostream>
#include <d3d_base/configFile.h>
#include <d3d_fusion/volumetricFusionTools.h>
#include <algorithm>

struct Hist {
    float counts[8];
};

D3D::Grid<float> runTVHist(D3D::Grid<Hist> dataCost, Eigen::Vector3f& minCorner, Eigen::Vector3f& size, Eigen::Matrix4f boxToGlobal, Eigen::Vector3f color, std::string vrmlOutputFile,
                                const int numIter,
                                const float lambda, const float theta)
{
    D3D::Grid<float> u(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth());
    for (unsigned z = 0; z < u.getDepth(); z++)
        for (unsigned y = 0; y < u.getHeight(); y++)
            for (unsigned x = 0; x < u.getWidth(); x++)
            {
                u(x,y,z) = 0.f;
            }

    D3D::Grid<float> uBar = u.clone();

    D3D::Grid<float> p1(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p2(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p3(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);

    const int xDim = u.getWidth();
    const int yDim = u.getHeight();
    const int zDim = u.getDepth();

    // uses diagonal preconditioned first order pirmal dual

    const float sigma = 0.99f/2.f;
    const float tau = 0.99f/6.f;
    const float centers[8] = {-1.f, -5.f/7.f, -3.f/7.f, -1.f/7.f, 1.f/7.f, 3.f/7.f, 5.f/7.f, 1.f};

    for (int c = 0; c < numIter; c++)
    {
        std::cout << ".";
        std::cout.flush();

        // update dual variables
#pragma omp parallel for
        for (int z = 0; z < zDim; z++)
        {
            int const Z1 = (z < zDim-1) ? (z+1) : z;
            for (int y = 0; y < yDim; y++)
            {
                int const Y1 = (y < yDim-1) ? (y+1) : y;
                for (int x = 0; x < xDim; x++)
                {
                    float const u0 = uBar(x,y,z);

                    int const X1 = (x < xDim -1) ? (x+1) : x;

                    float const u_x = (uBar(X1, y, z) - u0);
                    float const u_y = (uBar(x, Y1, z) - u0);
                    float const u_z = (uBar(x, y, Z1) - u0);

                    float const d_x = p1(x,y,z) + sigma*u_x;
                    float const d_y = p2(x,y,z) + sigma*u_y;
                    float const d_z = p3(x,y,z) + sigma*u_z;

                    float const tv = sqrtf(d_x*d_x + d_y*d_y + d_z*d_z);

                    float const denom = std::max(1.f, tv);

                    p1(x,y,z) = d_x/denom;
                    p2(x,y,z) = d_y/denom;
                    p3(x,y,z) = d_z/denom;
                }
            }
        }


        // update primal
#pragma omp parallel for
        for (int z = 0; z < zDim; z++)
        {
            int const Z0 = z-1;
            for (int y = 0; y < yDim; y++)
            {
                int const Y0 = y-1;
                for (int x = 0; x < xDim; x++)
                {
                    int const X0 = x-1;

                    float const div = (X0 < xDim -1 ? p1(x,y,z) : 0.0f) - (X0 >= 0 ? p1(X0, y, z) : 0.0f) +
                                      (Y0 < yDim -1 ? p2(x,y,z) : 0.0f) - (Y0 >= 0 ? p2(x, Y0, z) : 0.0f) +
                                      (Z0 < zDim -1 ? p3(x,y,z) : 0.0f) - (Z0 >= 0 ? p3(x, y, Z0) : 0.0f);

                    float const u0 = u(x,y,z);
                    float U = u(x,y,z) + tau*div;

//                    // check most likely bin, is in [1, 8] as u0 in [-1, 1]
//                    int mlBin = int(7.f * (u0 + 1.f) / 2.f + 1.f);

//                    float step = 0.f;
//                    for (int b = 0; b <= mlBin; ++b) {
//                        step -= float(dataCost(x, y, z).counts[b]);
//                    }
//                    for (int b = mlBin+1; b < 10; ++b) {
//                        step += float(dataCost(x, y, z).counts[b]);
//                    }

//                    float proxMin = u0 + tau*lambda*step;

                    float proxMin, step;

//                    if (proxMin <= centers[mlBin] || centers[mlBin+1] <= proxMin) {
                        // outside of range
                        bool found = false;
                        for (int bin = 0; bin < 7; bin++) {
//                            if (bin == mlBin)
//                                continue;
                            step = 0.f;
                            for (int b = 0; b <= bin; ++b) {
                                step -= dataCost(x, y, z).counts[b];
                            }
                            for (int b = bin+1; b < 8; ++b) {
                                step += dataCost(x, y, z).counts[b];
                            }

                            proxMin = U + tau*lambda*step;

                            if (centers[bin] < proxMin && proxMin < centers[bin+1]) {
                                // inside of range, val is the minimum
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            // minimum must be one of the bin centers
                            float proxMinEnergy = std::numeric_limits<float>::max();
                            for (int bin = 0; bin < 8; bin++) {
                                float dataterm = 0.f;
                                for (int i = 0; i < 8; ++i) {
                                    dataterm += dataCost(x, y, z).counts[i] * fabs(centers[bin]-centers[i]);
                                }
                                float proxEnergy = (U-centers[bin])*(U-centers[bin])/(2.f*tau) + lambda*dataterm;
                                if (proxEnergy < proxMinEnergy) {
                                    proxMinEnergy = proxEnergy;
                                    proxMin = centers[bin];
                                }
                            }
                        }
//                    }

                    U = proxMin;

                    u(x,y,z) = U;
                    uBar(x,y,z) = U + theta*(U - u0);

                }
            }
        }

        if ((c+1) % 50 == 0) {
            D3D::saveVolumeAsVRMLMesh(u, 0.0f, minCorner, size, boxToGlobal, color, vrmlOutputFile, true);
            std::cout << std::endl;
        }
    }
    return u;
}

int main(int argc, char* argv[])
{
    std::string configFile;
    std::string intDepthGridFile;
    std::string vrmlOutputFile;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("intDepthGridFile", boost::program_options::value<std::string>(&intDepthGridFile)->default_value("intCSDFGrid.dat"), "Depth map list file")
            ("configFile", boost::program_options::value<std::string>(&configFile)->default_value("conf.txt"), "Config file")
            ("vrmlOutputFile", boost::program_options::value<std::string>(&vrmlOutputFile)->default_value("tvHistModel.wrl"), "Output data file with integrated depth data.")
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
    // normlaize
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

    std::string hfNumIterStr = conf.get("HF_NUM_ITER");
    if (hfNumIterStr.empty())
    {
        D3D_THROW_EXCEPTION("Paramter HF_NUM_ITER is not specified in the config file.")
    }
    int ffNumIter = atoi(hfNumIterStr.c_str());

    std::string hfLambdaStr = conf.get("HF_LAMBDA");
    if (hfLambdaStr.empty())
    {
        D3D_THROW_EXCEPTION("Parameter HF_LAMBDA is not specified in the config file.")
    }
    float hfLambda = atof(hfLambdaStr.c_str());

    // load grid
    D3D::Grid<Hist> dataCost;
    dataCost.loadFromDataFile(intDepthGridFile);

    std::cout << "Data Cost Loaded: width = " << dataCost.getWidth() << ", height = " << dataCost.getHeight() << ", depth = " << dataCost.getDepth() << std::endl;

    if (bpResX != (int) dataCost.getWidth() || bpResY != (int) dataCost.getHeight() || bpResZ != (int) dataCost.getDepth())
    {
        D3D_THROW_EXCEPTION("Resolution specified in the config file does not match the dimension of the grid with integrated depth maps.")
    }

    Eigen::Vector3f minCorner;
    minCorner(0) = bpMinX;
    minCorner(1) = bpMinY;
    minCorner(2) = bpMinZ;
    Eigen::Vector3f size;
    size(0) = bpSizeX;
    size(1) = bpSizeY;
    size(2) = bpSizeZ;
    Eigen::Vector3f color;
    color(0) = 0.5;
    color(1) = 0.5;
    color(2) = 0.5;

    runTVHist(dataCost, minCorner, size, boxToGlobal, color, vrmlOutputFile, ffNumIter, hfLambda, 1);
}
