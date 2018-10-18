#ifndef GEODESIC_DOME_H
#define GEODESIC_DOME_H

#include <Eigen/Dense>
#include <vector>
#include <fstream>

namespace D3D
{
    class GeodesicDome
    {
    public:
        GeodesicDome();
        void generateIcosahedron();
        void generateTetrahedron();
        void toVRML(std::ofstream& os);
        void toVRMLPointsScaled(std::vector<double>& scaleFactors, std::ofstream& os);
        void subdivide();
        int numVertices();
        Eigen::Vector3d getVertex(int idx);
        void serialize(std::ostream& stream) const;
        void deserialize(std::istream& stream);

    private:
        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> faces;

    };
}

#endif // GEODESIC_DOME_H
