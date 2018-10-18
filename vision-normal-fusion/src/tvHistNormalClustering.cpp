#include <d3d_base/exception.h>
#include <d3d_base/configFile.h>
#include <d3d_base/grid.h>

#include <boost/program_options.hpp>

#include <fstream>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

struct NormalInfo {
    bool isIsotropic;
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3;
};

int main(int argc, char* argv[])
{
    std::string configFile;
    std::string normalsListFile;
    std::string outputFile;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("configFile", boost::program_options::value<std::string>(&configFile)->default_value("conf.txt"), "Config file")
            ("normalsListFile", boost::program_options::value<std::string>(&normalsListFile)->default_value("normalsFiles.txt"), "Normals list file")
            ("outputFile", boost::program_options::value<std::string>(&outputFile)->default_value("clusteredNormals.dat"), "Output file for clustered normals.")
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

    std::ifstream normalsStream;
    normalsStream.open(normalsListFile.c_str());

    if (!normalsStream.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open normals file list")
    }

    std::vector<std::string> normalsFileNames;
    std::string normalsFileName;
    while (normalsStream >> normalsFileName)
    {
        normalsFileNames.push_back(normalsFileName);
    }

    int ncNumClusters = conf.getAsInt("NC_NUM_CLUSTERS");
    int ncSpreadDist = conf.getAsInt("NC_SPREAD_DIST");
    float ncMinAngleDiff = conf.getAsFloat("NC_MIN_ANGLE_DIFF");

    D3D::Grid< std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > > normals(bpResX, bpResY, bpResZ);
//    D3D::Grid<NormalInfo> normals(bpResX, bpResY, bpResZ);

    for (unsigned int i = 0; i < normalsFileNames.size(); ++i) {
        D3D::Grid<Eigen::Vector3f> normalFile;
        normalFile.loadFromDataFile(normalsFileNames[i]);

        for (int z = 0; z < bpResZ; z++) {
            for (int y = 0; y < bpResY; y++) {
                for (int x = 0; x < bpResX; x++) {
                    Eigen::Vector3f normal = normalFile(x,y,z);
                    if (normal.norm() > 0.) {
                        normal /= normal.norm();
                        normals(x, y, z).push_back(normal);
                    }
                }
            }
        }
    }


    const float cosMinAngleDiff = cos(ncMinAngleDiff);

    D3D::Grid<std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > > allClusters(bpResX, bpResY, bpResZ);

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >::const_iterator nIter;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >::iterator cIter;
    std::vector<int>::iterator aIter;

    int countOccupied = 0;
    std::vector<int> clusterCount(1, 0);
//    pcl::PointCloud<pcl::PointNormal> clusterLines;


    for (int z = 0; z < bpResZ; z++) {
        std::cout << " " << z << std::flush;
        for (int y = 0; y < bpResY; y++) {
            for (int x = 0; x < bpResX; x++) {

                int sXmin = x-ncSpreadDist >= 0 ? ncSpreadDist : 0; //x;
                int sXmax = x+ncSpreadDist < bpResX ? ncSpreadDist : 0; //bpResX-1-x;

                int sYmin = y-ncSpreadDist >= 0 ? ncSpreadDist : 0;//y;
                int sYmax = y+ncSpreadDist < bpResY ? ncSpreadDist : 0;//bpResY-1-y;

                int sZmin = z-ncSpreadDist >= 0 ? ncSpreadDist : 0;//z;
                int sZmax = z+ncSpreadDist < bpResZ ? ncSpreadDist : 0;//bpResZ-1-z;

                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > clusters(1);

                // Initialize assignments
                int numNormals = 0;
                for (int i = x-sXmin; i <= x+sXmax; ++i)
                    for (int j = y-sYmin; j <= y+sYmax; ++j)
                        for (int k = z-sZmin; k <= z+sZmax; ++k) {
                            numNormals += normals(i, j, k).size();
                        }
                std::vector<int> assignments(numNormals, 0);
                if (numNormals == 0) {
                    // No normals
                    // clusterCount(x, y, z) = 0;
                    continue;
                }

                bool newClusterNeeded = true;
                while (newClusterNeeded) {
                    // Assign normals to clusters
                    aIter = assignments.begin();
                    for (int i = x-sXmin; i <= x+sXmax; ++i)
                        for (int j = y-sYmin; j <= y+sYmax; ++j)
                            for (int k = z-sZmin; k <= z+sZmax; ++k) {
                                nIter = normals(i, j, k).begin();
                                for (; nIter != normals(i, j, k).end(); ++nIter, ++aIter) {
                                    float maxDotProd = -2.;
                                    int minClusterInd = -1;
                                    for (unsigned int c = 0; c < clusters.size(); ++c) {
                                        if (nIter->dot(clusters[c]) > maxDotProd) {
                                            maxDotProd = nIter->dot(clusters[c]);
                                            minClusterInd = c;
                                        }
                                    }
                                    *aIter = minClusterInd;
                                }
                            }

                    // Compute cluster centers
                    for (cIter = clusters.begin(); cIter != clusters.end(); ++cIter) {
                        (*cIter)(0) = 0.;
                        (*cIter)(1) = 0.;
                        (*cIter)(2) = 0.;
                    }
                    aIter = assignments.begin();
                    for (int i = x-sXmin; i <= x+sXmax; ++i)
                        for (int j = y-sYmin; j <= y+sYmax; ++j)
                            for (int k = z-sZmin; k <= z+sZmax; ++k) {
                                nIter = normals(i, j, k).begin();
                                for (; nIter != normals(i, j, k).end(); ++nIter, ++aIter) {
                                    clusters[*aIter] += *nIter;
                                }
                            }
                    for (unsigned int i = 0; i < clusters.size(); ++i) {
                        clusters[i] /= clusters[i].norm();
                    }

                    // Check if more clusters are needed
                    newClusterNeeded = false;
                    aIter = assignments.begin();
                    float minCosAngle = 2.f;
                    int minAssignmentInd = -1;
                    Eigen::Vector3f minNormal;
                    std::vector<int> outsideClusterCounts(clusters.size(), 0);
                    std::vector<int> allClusterCounts(clusters.size(), 0);
                    for (int i = x-sXmin; i <= x+sXmax; ++i)
                        for (int j = y-sYmin; j <= y+sYmax; ++j)
                            for (int k = z-sZmin; k <= z+sZmax; ++k) {
                                nIter = normals(i, j, k).begin();
                                for (; nIter != normals(i, j, k).end(); ++nIter, ++aIter) {
                                    float cosAngle = nIter->dot(clusters[*aIter]);
                                    if (cosAngle < cosMinAngleDiff) {
                                        outsideClusterCounts[*aIter]++;
                                        if (cosAngle < minCosAngle) {
                                            minCosAngle = cosAngle;
                                            minNormal = *nIter;
                                            minAssignmentInd = int(aIter-assignments.begin());
//                                            newClusterNeeded = true;
                                        }
                                    }
                                    allClusterCounts[*aIter]++;
                                }
                    }

                    // Soft clustering 90% of the normals have to fall inside the cluster
                    for (unsigned int i = 0; i < outsideClusterCounts.size(); ++i) {
                        if (outsideClusterCounts[i]/float(allClusterCounts[i]) > 0.1) {
                            newClusterNeeded = true;
                            assignments[minAssignmentInd] = clusters.size();
                            clusters.push_back(minNormal);
                            break;
                        }
                    }
                }

                int numClusters = clusters.size();
                allClusters(x, y, z) = clusters;

//                    for (int i = 0; i < numClusters; ++i) {
//                        pcl::PointNormal pt;
//                        pt.x = x+0.5;
//                        pt.y = y+0.5;
//                        pt.z = z+0.5;
//                        pt.normal_x = clusters[i](0);
//                        pt.normal_y = clusters[i](1);
//                        pt.normal_z = clusters[i](2);
//                        clusterLines.push_back(pt);
//                    }

                if (unsigned(numClusters) >= clusterCount.size()) {
                    clusterCount.resize(numClusters+1, 0);
                }
                clusterCount[numClusters]++;

                countOccupied++;
            }
        }
    }

    std::cout << std::endl << std::endl << "Cluster statistics: " << std::endl;
    for (unsigned int i = 1; i < clusterCount.size(); ++i) {
        std::cout << i << ": " << clusterCount[i] << " (" << clusterCount[i] / float(countOccupied)*100 << "%)" << std::endl;
    }
//    pcl::io::savePLYFile("clusterLines.ply", clusterLines);

    D3D::Grid<NormalInfo> clusteredNormals(bpResX, bpResY, bpResZ);

    for (int z = 0; z < bpResZ; z++) {
        for (int y = 0; y < bpResY; y++) {
            for (int x = 0; x < bpResX; x++) {
                int numClusters = allClusters(x,y,z).size();
                if (numClusters > 0 && numClusters <= ncNumClusters) {
                    clusteredNormals(x,y,z).isIsotropic = false;
                    clusteredNormals(x,y,z).n1 = allClusters(x,y,z)[0];
                    clusteredNormals(x,y,z).n2 = allClusters(x,y,z)[std::min(1, numClusters-1)];
                    clusteredNormals(x,y,z).n3 = allClusters(x,y,z)[std::min(2, numClusters-1)];
                } else {
                    clusteredNormals(x,y,z).isIsotropic = true;
                }
            }
        }
    }

    clusteredNormals.saveAsDataFile(outputFile);
}
