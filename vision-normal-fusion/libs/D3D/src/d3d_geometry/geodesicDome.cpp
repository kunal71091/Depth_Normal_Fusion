#include "geodesicDome.h"

#include <map>

namespace D3D
{

GeodesicDome::GeodesicDome()
{
    generateIcosahedron();
}

void GeodesicDome::generateIcosahedron()
{
    vertices.clear();
    faces.clear();

    // create new points
    float tao = 1.61803399; // golden ratio constant

    Eigen::Vector3d point;
    point(0) = 1; point(1) = tao; point(2) = 0;
    vertices.push_back(point);
    point(0) = -1; point(1) = tao; point(2) = 0;
    vertices.push_back(point);
    point(0) = 1; point(1) = -tao; point(2) = 0;
    vertices.push_back(point);
    point(0) = -1; point(1) = -tao; point(2) = 0;
    vertices.push_back(point);
    point(0) = 0; point(1) = 1; point(2) = tao;
    vertices.push_back(point);
    point(0) = 0; point(1) = -1; point(2) = tao;
    vertices.push_back(point);
    point(0) = 0; point(1) = 1; point(2) = -tao;
    vertices.push_back(point);
    point(0) = 0; point(1) = -1; point(2) = -tao;
    vertices.push_back(point);
    point(0) = tao; point(1) = 0; point(2) = 1;
    vertices.push_back(point);
    point(0) = -tao; point(1) = 0; point(2) = 1;
    vertices.push_back(point);
    point(0) = tao; point(1) = 0; point(2) = -1;
    vertices.push_back(point);
    point(0) = -tao; point(1) = 0; point(2) = -1;
    vertices.push_back(point);

    // normalize points
    for (unsigned int i=0; i < vertices.size(); i++) {
        vertices[i] /= vertices[i].norm();
    }

    // add the faces
    Eigen::Vector3i face;
    face << 0, 1, 4;
    faces.push_back(face);
    face << 1, 9, 4;
    faces.push_back(face);
    face << 4, 9, 5;
    faces.push_back(face);
    face << 5, 9, 3;
    faces.push_back(face);
    face << 2, 3, 7;
    faces.push_back(face);
    face << 3, 2, 5;
    faces.push_back(face);
    face << 7, 10, 2;
    faces.push_back(face);
    face << 0, 8, 10;
    faces.push_back(face);
    face << 0, 4, 8;
    faces.push_back(face);
    face << 8, 2, 10;
    faces.push_back(face);
    face << 8, 4, 5;
    faces.push_back(face);
    face << 8, 5, 2;
    faces.push_back(face);
    face << 1, 0, 6;
    faces.push_back(face);
    face << 11, 1, 6;
    faces.push_back(face);
    face << 3, 9, 11;
    faces.push_back(face);
    face << 6, 10, 7;
    faces.push_back(face);
    face << 3, 11, 7;
    faces.push_back(face);
    face << 11, 6, 7;
    faces.push_back(face);
    face << 6, 0, 10;
    faces.push_back(face);
    face << 9, 1, 11;
    faces.push_back(face);
}

void GeodesicDome::generateTetrahedron()
{
    vertices.clear();
    faces.clear();

    // create new points
    const double sqrt2 = std::sqrt(2);

    Eigen::Vector3d point;
    point(0) = 1; point(1) = 0; point(2) = -1/sqrt2;
    vertices.push_back(point);
    point(0) = -1; point(1) = 0; point(2) = -1/sqrt2;
    vertices.push_back(point);
    point(0) = 0; point(1) = 1; point(2) = 1/sqrt2;
    vertices.push_back(point);
    point(0) = 0; point(1) = -1; point(2) = 1/sqrt2;
    vertices.push_back(point);


    // normalize points
    for (unsigned int i=0; i < vertices.size(); i++) {
        vertices[i] /= vertices[i].norm();
    }

    // add the faces
    Eigen::Vector3i face;
    face << 0, 2, 3;
    faces.push_back(face);
    face << 0, 1, 2;
    faces.push_back(face);
    face << 1, 3, 2;
    faces.push_back(face);
    face << 3, 1, 0;
    faces.push_back(face);
}

void GeodesicDome::toVRML(std::ofstream &os)
{
    os << "#VRML V2.0 utf8" << std::endl << std::endl;

    os << "Shape {" << std::endl;
    os << "geometry IndexedLineSet {" << std::endl;
    os << "coord Coordinate {" << std::endl;
    os << "point [" << std::endl;

    for (unsigned int i = 0; i < vertices.size(); i++) {
        os << vertices[i](0) << " " << vertices[i](1) << " " << vertices[i](2) << std::endl;
    }

    os << "]" << std::endl;
    os << "}" << std::endl;
    os << "coordIndex [" << std::endl;
    for (unsigned int i=0; i < faces.size(); i++) {
        os << faces[i](0) << ", " << faces[i](1) << ", -1," << std::endl;
        os << faces[i](1) << ", " << faces[i](2) << ", -1," << std::endl;
        os << faces[i](2) << ", " << faces[i](0) << ", -1," << std::endl;
    }

    os << "]" << std::endl;
    os << "}" << std::endl;
    os << "}" << std::endl;
}

void GeodesicDome::toVRMLPointsScaled(std::vector<double> &scaleFactors, std::ofstream &os)
{
    os << "#VRML V2.0 utf8" << std::endl << std::endl;

    os << "Shape {" << std::endl;
    os << "geometry IndexedLineSet {" << std::endl;
    os << "coord Coordinate {" << std::endl;
    os << "point [" << std::endl;

    for (unsigned int i = 0; i < vertices.size(); i++) {
        os << scaleFactors[i]*vertices[i](0) << " " << scaleFactors[i]*vertices[i](1) << " " << scaleFactors[i]*vertices[i](2) << std::endl;
    }

    os << "]" << std::endl;
    os << "}" << std::endl;
    os << "coordIndex [" << std::endl;
    for (unsigned int i=0; i < faces.size(); i++) {
        os << faces[i](0) << ", " << faces[i](1) << ", -1," << std::endl;
        os << faces[i](1) << ", " << faces[i](2) << ", -1," << std::endl;
        os << faces[i](2) << ", " << faces[i](0) << ", -1," << std::endl;
    }

    os << "]" << std::endl;
    os << "}" << std::endl;
    os << "}" << std::endl;
}

void GeodesicDome::subdivide()
{
    std::vector<Eigen::Vector3i> newTriangles;
    std::map<std::pair<int, int>, int> midpoints;

    Eigen::Vector3d point;
    for (unsigned int i = 0; i < faces.size(); i++)
    {
        int vj[3];

        for (int j = 0; j < 3; j++)
        {
            if (midpoints.count(std::make_pair(faces[i](j), faces[i]((j+1)%3))))
            {
                vj[j] = midpoints[std::make_pair(faces[i](j), faces[i]((j+1)%3))];
            }
            else if(midpoints.count(std::make_pair(faces[i]((j+1)%3), faces[i](j))))
            {
                vj[j] = midpoints[std::make_pair(faces[i]((j+1)%3), faces[i](j))];
            }
            else
            {
                // not already computed needs to compute midpoint
                point = 0.5*(vertices[faces[i](j)] + vertices[faces[i]((j+1)%3)]);
                point.normalize();

                vj[j] = vertices.size();
                midpoints[std::make_pair(faces[i](j), faces[i]((j+1)%3))] = vj[j];
                vertices.push_back(point);
            }
        }

        Eigen::Vector3i triangle;
        triangle << faces[i](0), vj[0], vj[2];
        newTriangles.push_back(triangle);
        triangle << vj[0], faces[i](1), vj[1];
        newTriangles.push_back(triangle);
        triangle << vj[1], faces[i](2), vj[2];
        newTriangles.push_back(triangle);
        triangle << vj[0], vj[1], vj[2];
        newTriangles.push_back(triangle);
    }

    faces = newTriangles;
}

int GeodesicDome::numVertices()
{
    return vertices.size();
}

Eigen::Vector3d GeodesicDome::getVertex(int idx)
{
    return vertices[idx];
}

void GeodesicDome::serialize(std::ostream& stream) const
{
    int numVert = vertices.size();
    stream.write((char*) &numVert, sizeof(int));

    for (int i = 0; i < numVert; i++)
    {
        stream.write((char*) &(vertices[i](0)), sizeof(double));
        stream.write((char*) &(vertices[i](1)), sizeof(double));
        stream.write((char*) &(vertices[i](2)), sizeof(double));
    }

    int numTriangles = faces.size();
    stream.write((char *) &numTriangles, sizeof(int));

    for (int i = 0; i < numTriangles; i++)
    {
        stream.write((char*) &(faces[i](0)), sizeof(int));
        stream.write((char*) &(faces[i](1)), sizeof(int));
        stream.write((char*) &(faces[i](2)), sizeof(int));
    }
}

void GeodesicDome::deserialize(std::istream &stream)
{
    int numVert;
    stream.read((char*) &numVert, sizeof(int));
    vertices.resize(numVert);

    for (int i = 0; i < numVert; i++)
    {
        stream.read((char*) &(vertices[i](0)), sizeof(double));
        stream.read((char*) &(vertices[i](1)), sizeof(double));
        stream.read((char*) &(vertices[i](2)), sizeof(double));
    }

    int numTriangles;
    stream.read((char*) &numTriangles, sizeof(int));
    faces.resize(numTriangles);

    for (int i = 0; i < numTriangles; i++)
    {
        stream.read((char *) &(faces[i](0)), sizeof(int));
        stream.read((char *) &(faces[i](1)), sizeof(int));
        stream.read((char *) &(faces[i](2)), sizeof(int));
    }
}

}
