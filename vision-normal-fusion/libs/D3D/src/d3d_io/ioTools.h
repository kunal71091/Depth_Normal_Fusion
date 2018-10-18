#ifndef IOTOOLS_H
#define IOTOOLS_H

#include <string>
#include <vector>
#include <Eigen/Dense>


namespace D3D
{
    bool is_little_endian();
    std::string extractBaseFileName(const std::string& fullFileName);
    std::string extractFileName(const std::string& fullFileName);
    std::string extractPath(const std::string& fullFileName);

    // reads a simple obj file vertice indices are returned 0 based
    void readObjFile(const std::string& fileName, std::vector<Eigen::Vector3d>& vertices, std::vector<std::vector<int> >& faces);
}


#endif // IOTOOLS_H
