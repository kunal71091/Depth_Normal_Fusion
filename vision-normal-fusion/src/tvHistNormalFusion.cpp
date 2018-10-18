#include <boost/program_options.hpp>
#include <string>
#include <d3d_base/grid.h>
#include <iostream>
#include <d3d_base/configFile.h>
#include <d3d_fusion/volumetricFusionTools.h>
#include <algorithm>

void projectToHalspaceSpereIntersection(float& x, float& y, float& z, float nx, float ny, float nz, float dist, float radius)
{
    // check if point inside halfspace
    float pd = nx*x + ny*y + nz*z;
    if (pd < dist) {
        // project to sphere
        float norm = std::sqrt(x*x + y*y + z*z);
        float denom = std::max(1.f, norm/radius);
        x /= denom;
        y /= denom;
        z /= denom;
    } else {
        // project on halfplane
        float distDiff = pd-dist;
        x -= distDiff*nx;
        y -= distDiff*ny;
        z -= distDiff*nz;

        // check if inside sphere, if so we are done, otherwise project on circle (assuming radius > dist)
        float norm = std::sqrt(x*x + y*y + z*z);
        if (norm > radius) {
            // precompute for more efficient implemenetation
            float cicleRadius = std::sqrt(radius*radius - dist*dist);

            // shift to origin
            x -= dist*nx;
            y -= dist*ny;
            z -= dist*nz;

            // rescale such that norm = circleRadius
            norm = std::sqrt(x*x + y*y + z*z);
            x *= cicleRadius/norm;
            y *= cicleRadius/norm;
            z *= cicleRadius/norm;

            // shift to plane
            x += dist*nx;
            y += dist*ny;
            z += dist*nz;
        }
    }
}

struct Hist {
    float counts[8];
};

struct NormalInfo {
    bool isIsotropic;
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3;
};

D3D::Grid<float> runTVHist(D3D::Grid<Hist>& dataCost, D3D::Grid<NormalInfo>& normals, Eigen::Vector3f& minCorner, Eigen::Vector3f& size,
                           Eigen::Matrix4f boxToGlobal, Eigen::Vector3f color, std::string vrmlOutputFile,
                           const int numIter, const float lambda, const float theta)
{
    D3D::Grid<float> u(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth());
    for (unsigned z = 0; z < u.getDepth(); z++)
        for (unsigned y = 0; y < u.getHeight(); y++)
            for (unsigned x = 0; x < u.getWidth(); x++)
            {
                u(x,y,z) = 0.f;
            }

    D3D::Grid<float> uBar = u.clone();

    D3D::Grid<float> lambda11(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> lambda12(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> lambda13(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> lambda21(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> lambda22(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> lambda23(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);

    D3D::Grid<float> lambdaBar11 = lambda11.clone();
    D3D::Grid<float> lambdaBar12 = lambda12.clone();
    D3D::Grid<float> lambdaBar13 = lambda13.clone();
    D3D::Grid<float> lambdaBar21 = lambda21.clone();
    D3D::Grid<float> lambdaBar22 = lambda22.clone();
    D3D::Grid<float> lambdaBar23 = lambda23.clone();

    // dual variables
    D3D::Grid<float> p11(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p12(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p13(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p21(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p22(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p23(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p31(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p32(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);
    D3D::Grid<float> p33(dataCost.getWidth(), dataCost.getHeight(), dataCost.getDepth(), 0);

    const int xDim = u.getWidth();
    const int yDim = u.getHeight();
    const int zDim = u.getDepth();

    // uses diagonal preconditioned first order pirmal dual

    const float sigma = 0.99f/3.f;
    const float tau = 0.99f/12.f;
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

                    float& p1w = p11(x,y,z);
                    float& p1h = p12(x,y,z);
                    float& p1d = p13(x,y,z);
                    float& p2w = p21(x,y,z);
                    float& p2h = p22(x,y,z);
                    float& p2d = p23(x,y,z);
                    float& p3w = p31(x,y,z);
                    float& p3h = p32(x,y,z);
                    float& p3d = p33(x,y,z);

                    p1w += sigma*(u_x + lambdaBar11(x,y,z));
                    p1h += sigma*(u_y + lambdaBar12(x,y,z));
                    p1d += sigma*(u_z + lambdaBar13(x,y,z));

                    p2w += sigma*(-lambdaBar11(x,y,z) + lambdaBar21(x,y,z));
                    p2h += sigma*(-lambdaBar12(x,y,z) + lambdaBar22(x,y,z));
                    p2d += sigma*(-lambdaBar13(x,y,z) + lambdaBar23(x,y,z));

                    p3w += sigma*(-lambdaBar21(x,y,z));
                    p3h += sigma*(-lambdaBar22(x,y,z));
                    p3d += sigma*(-lambdaBar23(x,y,z));


                    if (normals(x,y,z).isIsotropic) {
                        float tv = sqrtf(p1w*p1w + p1h*p1h + p1d*p1d);
                        float denom = std::max(1.f, tv/2.f);
                        p1w /= denom;
                        p1h /= denom;
                        p1d /= denom;

                        tv = sqrtf(p2w*p2w + p2h*p2h + p2d*p2d);
                        denom = std::max(1.f, tv/2.f);
                        p2w /= denom;
                        p2h /= denom;
                        p2d /= denom;

                        tv = sqrtf(p3w*p3w + p3h*p3h + p3d*p3d);
                        denom = std::max(1.f, tv/2.f);
                        p3w /= denom;
                        p3h /= denom;
                        p3d /= denom;
                    } else {
                        Eigen::Vector3f n1 = -normals(x,y,z).n1;
                        Eigen::Vector3f n2 = -normals(x,y,z).n2;
                        Eigen::Vector3f n3 = -normals(x,y,z).n3;
                        projectToHalspaceSpereIntersection(p1w, p1h, p1d, n1(0), n1(1), n1(2), 0.5, 2.0);
                        projectToHalspaceSpereIntersection(p2w, p2h, p2d, n2(0), n2(1), n2(2), 0.5, 2.0);
                        projectToHalspaceSpereIntersection(p3w, p3h, p3d, n3(0), n3(1), n3(2), 0.5, 2.0);
                    }
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

                    float const div = (X0 < xDim -1 ? p11(x,y,z) : 0.0f) - (X0 >= 0 ? p11(X0, y, z) : 0.0f) +
                                      (Y0 < yDim -1 ? p12(x,y,z) : 0.0f) - (Y0 >= 0 ? p12(x, Y0, z) : 0.0f) +
                                      (Z0 < zDim -1 ? p13(x,y,z) : 0.0f) - (Z0 >= 0 ? p13(x, y, Z0) : 0.0f);

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

                    float const L11 = lambda11(x,y,z) - tau*(p11(x,y,z) - p21(x,y,z));
                    float const L12 = lambda12(x,y,z) - tau*(p12(x,y,z) - p22(x,y,z));
                    float const L13 = lambda13(x,y,z) - tau*(p13(x,y,z) - p23(x,y,z));

                    lambdaBar11(x,y,z) = L11 + theta*(L11 - lambda11(x,y,z));
                    lambdaBar12(x,y,z) = L12 + theta*(L12 - lambda12(x,y,z));
                    lambdaBar13(x,y,z) = L13 + theta*(L13 - lambda13(x,y,z));

                    lambda11(x,y,z) = L11;
                    lambda12(x,y,z) = L12;
                    lambda13(x,y,z) = L13;

                    float const L21 = lambda21(x,y,z) - tau*(p21(x,y,z) - p31(x,y,z));
                    float const L22 = lambda22(x,y,z) - tau*(p22(x,y,z) - p32(x,y,z));
                    float const L23 = lambda23(x,y,z) - tau*(p23(x,y,z) - p33(x,y,z));

                    lambdaBar21(x,y,z) = L21 + theta*(L21 - lambda21(x,y,z));
                    lambdaBar22(x,y,z) = L22 + theta*(L22 - lambda22(x,y,z));
                    lambdaBar23(x,y,z) = L23 + theta*(L23 - lambda23(x,y,z));

                    lambda21(x,y,z) = L21;
                    lambda22(x,y,z) = L22;
                    lambda23(x,y,z) = L23;
                }
            }
        }

        if ((c+1) % 50 == 0) {
            D3D::saveVolumeAsVRMLMesh(u, 0.0f, minCorner, size, boxToGlobal, color, vrmlOutputFile, true);
            u.saveAsDataFile("tvHistNormalFusionU.dat");
            std::cout << std::endl;

        }
    }
    return u;
}

int main(int argc, char* argv[])
{
    std::string configFile;
    std::string intDepthGridFile;
    std::string clusteredNormalsFile;
    std::string vrmlOutputFile;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("intDepthGridFile", boost::program_options::value<std::string>(&intDepthGridFile)->default_value("intCSDFGrid.dat"), "Depth map list file")
            ("clusteredNormalsFile", boost::program_options::value<std::string>(&clusteredNormalsFile)->default_value("clusteredNormals.dat"), "Clustered normals file")
            ("configFile", boost::program_options::value<std::string>(&configFile)->default_value("conf.txt"), "Config file")
            ("vrmlOutputFile", boost::program_options::value<std::string>(&vrmlOutputFile)->default_value("tvHistNormalModel.wrl"), "Output data file with integrated depth data.")
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

    // load clustered normals
    D3D::Grid<NormalInfo> clusteredNormals;
    clusteredNormals.loadFromDataFile(clusteredNormalsFile);

    std::cout << "Clustered Normals Loaded: width = " << clusteredNormals.getWidth() << ", height = " << clusteredNormals.getHeight() << ", depth = " << clusteredNormals.getDepth() << std::endl;

    if (bpResX != (int) clusteredNormals.getWidth() || bpResY != (int) clusteredNormals.getHeight() || bpResZ != (int) clusteredNormals.getDepth())
    {
        D3D_THROW_EXCEPTION("Resolution specified in the config file does not match the dimension of the grid with the clustered normals.")
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

    runTVHist(dataCost, clusteredNormals, minCorner, size, boxToGlobal, color, vrmlOutputFile, ffNumIter, hfLambda, 1);
}
