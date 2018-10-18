#include "fishEyeDepthMap.h"

#include <d3d_base/common.h>

#include <string>

#include <d3d_base/colorMapJet.h>
#include <d3d_base/grid.h>

#include <opencv2/highgui/highgui.hpp>

using namespace D3D;
using namespace std;

template <typename T, typename U>
FishEyeDepthMap<T, U>::FishEyeDepthMap()
{
    width = 0;
    height = 0;
}

template <typename T, typename U>
FishEyeDepthMap<T, U>::FishEyeDepthMap(unsigned int width, unsigned int height, const FishEyeCameraMatrix<U> &cam)
{
    this->width = width;
    this->height = height;
    this->cam = cam;

    T* depths = new T[width*height];
    this->depths.reset(depths);
}

template <typename T, typename U>
T* FishEyeDepthMap<T, U>::getDataPtr()
{
    return &(depths[0]);
}

template <typename T, typename U>
unsigned int FishEyeDepthMap<T, U>::getWidth() const
{
    return width;
}

template <typename T, typename U>
unsigned int FishEyeDepthMap<T, U>::getHeight() const
{
    return height;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::display(T minZ, T maxZ, int displayTime, const char *windowName)
{
    cv::Mat_<T> depthsMat(height, width, getDataPtr());
    cv::imshow(windowName, (depthsMat-minZ)/(maxZ-minZ));
    cv::waitKey(displayTime);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::displayColored(T minZ, T maxZ, int displayTime, const char *windowName)
{

    cv::Mat colDepth = cv::Mat::zeros(height, width, CV_8UC3);

     for (int y = 0; y < colDepth.rows; y++)
     {
         unsigned char* pixel = colDepth.ptr<unsigned char>(y);
         for (int x = 0; x < colDepth.cols; ++x)
         {
             const T depth = (*this)(x,y);
             if (depth > 0)
             {
                 int idx = (int) round(std::max((T) 0, std::min((depth - minZ)/(maxZ-minZ), (T) 1) * (T) 255));

                 pixel[0] = (unsigned char) floor(colorMapJet[idx][2] * 255.0f + 0.5f);
                 pixel[1] = (unsigned char) floor(colorMapJet[idx][1] * 255.0f + 0.5f);
                 pixel[2] = (unsigned char) floor(colorMapJet[idx][0] * 255.0f + 0.5f);
             }

             pixel += 3;
         }
     }
    cv::imshow(windowName, colDepth);
    cv::waitKey(displayTime);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::displayInvDepth(T minZ, T maxZ, int displayTime, const char *windowName)
{
    cv::Mat_<T> depthsMat(height, width, getDataPtr());
    cv::Mat_<T> invDepthsMat(height, width);
    for (unsigned int y = 0; y < height; y++)
    {
        for (unsigned int x = 0; x < width; x++)
        {
            const T depth = depthsMat[y][x];
            if (depth > 0)
            {
                invDepthsMat[y][x] = (1/depthsMat[y][x]-1/maxZ)/(1/minZ - 1/maxZ);
            }
            else
            {
                invDepthsMat[y][x] = 0;
            }
        }
    }
    cv::imshow(windowName, invDepthsMat);
    cv::waitKey(displayTime);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::displayInvDepthColored(T minZ, T maxZ, int displayTime, const char *windowName)
{
    cv::Mat colInvDepth = cv::Mat::zeros(height, width, CV_8UC3);

     for (int y = 0; y < colInvDepth.rows; y++)
     {
         unsigned char* pixel = colInvDepth.ptr<unsigned char>(y);
         for (int x = 0; x < colInvDepth.cols; ++x)
         {
             const T depth = (*this)(x,y);
             if (depth > 0)
             {
                 int idx = (int) round(std::max((T) 0, std::min(1/depth - 1/maxZ, 1/minZ - 1/maxZ) / (1/minZ - 1/maxZ)) * (T) 255);

                 pixel[0] = (int) round(colorMapJet[idx][2] * 255.0f);
                 pixel[1] = (int) round(colorMapJet[idx][1] * 255.0f);
                 pixel[2] = (int) round(colorMapJet[idx][0] * 255.0f);
             }

             pixel += 3;
         }
     }
    cv::imshow(windowName, colInvDepth);
    cv::waitKey(displayTime);
}


template <typename T, typename U>
T& FishEyeDepthMap<T, U>::operator() (int x, int y)
{
    return depths[y*width + x];
}

template <typename T, typename U>
const T& FishEyeDepthMap<T, U>::operator() (int x, int y) const
{
    return depths[y*width + x];
}

template <typename T, typename U>
const FishEyeCameraMatrix<U>& FishEyeDepthMap<T,U>::getCam() const
{
    return cam;
}

template <typename T, typename U>
Eigen::Matrix<U, 4, 1> FishEyeDepthMap<T, U>::unproject(int x, int y) const
{
    U depth = (U) (*this)(x, y);
    if (depth <= 0)
    {
        return Eigen::Matrix<U, 4, 1>::Zero();
    }

    return getCam().unprojectPoint((U) x, (U) y, (U) depth);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::pointCloudToVRML(std::ofstream& os, U maxDist)
{
    if (!os.is_open())
    {
        D3D_THROW_EXCEPTION("Outputstream not open.")
    }

    D3D::Grid<Eigen::Matrix<U, 4, 1> > reprojPoints(width, height);
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            reprojPoints(x,y) = unproject(x,y);
            if (maxDist > 0 && reprojPoints(x,y)(3) == 1)
            {
                U dist = (reprojPoints(x,y).topRows(3) - cam.getC()).norm();
                if (maxDist < dist)
                {
                    reprojPoints(x,y)(0) = 0;
                    reprojPoints(x,y)(1) = 0;
                    reprojPoints(x,y)(2) = 0;
                    reprojPoints(x,y)(3) = 0;
                }
            }
        }

    os << "#VRML V2.0 utf8" << std::endl;
    os << "Shape {" << std::endl;
    os << "     appearance Appearance {" << std::endl;
    os << "         material Material { " << std::endl;
    os << "             diffuseColor     0.5 0.5 0.5" << std::endl;
    os << "         }" << std::endl;
    os << "     }" << std::endl;
    os << "     geometry PointSet {" << std::endl;
    os << "       coord Coordinate {" << std::endl;
    os << "           point [" << std::endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," << std::endl;
    os << "           ]" << std::endl;
    os << "       }" << std::endl;
    os << "   }" << std::endl;
    os << "}" << std::endl;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::pointCloudColoredToVRML(std::ofstream& os, cv::Mat image, U maxDist)
{
    if (!os.is_open())
    {
        D3D_THROW_EXCEPTION("Outputstream not open.")
    }

    Grid<Eigen::Matrix<U, 4, 1> > reprojPoints(width, height);
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            reprojPoints(x,y) = unproject(x,y);
            if (maxDist > 0 && reprojPoints(x,y)(3) == 1)
            {
                U dist = (reprojPoints(x,y).topRows(3) - cam.getC()).norm();
                if (maxDist < dist)
                {
                    reprojPoints(x,y)(0) = 0;
                    reprojPoints(x,y)(1) = 0;
                    reprojPoints(x,y)(2) = 0;
                    reprojPoints(x,y)(3) = 0;
                }
            }
        }

    os << "#VRML V2.0 utf8" << std::endl;
    os << "Shape {" <<  std::endl;
    os << "     appearance Appearance {" <<  std::endl;
    os << "         material Material { " <<  std::endl;
    os << "             diffuseColor     0.5 0.5 0.5" <<  std::endl;
    os << "         }" <<  std::endl;
    os << "     }" <<  std::endl;
    os << "     geometry PointSet {" <<  std::endl;
    os << "       coord Coordinate {" <<  std::endl;
    os << "           point [" <<  std::endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," <<  std::endl;
    os << "           ]" <<  std::endl;
    os << "       }" <<  std::endl;
    os << "       color Color {" <<  std::endl;
    os << "         color [" <<  std::endl;
    if (image.channels() == 1)
    {
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            os << "           " << image.at<unsigned char>(y,x)/255.0 << " " << image.at<unsigned char>(y,x)/255.0 << " " << image.at<unsigned char>(y,x)/255.0 << "," <<  std::endl;
        }
    }
    else if (image.channels() == 3)
    {
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
            {
                os << "           " << image.at<unsigned char>(y,3*x+2)/255.0 << " " << image.at<unsigned char>(y,3*x+1)/255.0 << " " << image.at<unsigned char>(y,3*x)/255.0 << "," <<  std::endl;
            }
    }
    os << "         ]" <<  std::endl;
    os << "       }" <<  std::endl;
    os << "   }" <<  std::endl;
    os << "}" <<  std::endl;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::setCam(const FishEyeCameraMatrix<U>& cam)
{
    this->cam = cam;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::saveInvDepthAsColorImage(std::string fileName, T minZ, T maxZ)
{

   cv::Mat colDepth = cv::Mat::zeros(height, width, CV_8UC3);

    for (int y = 0; y < colDepth.rows; y++)
    {
        unsigned char* pixel = colDepth.ptr<unsigned char>(y);
        for (int x = 0; x < colDepth.cols; ++x)
        {
            const T depth = (*this)(x,y);
            if (depth > 0)
            {
                int idx = (int) round(std::max((T) 0, std::min(1/depth - 1/maxZ, 1/minZ - 1/maxZ) / (1/minZ - 1/maxZ)) * (T) 255);

                pixel[0] = (int) round(colorMapJet[idx][2] * 255.0f);
                pixel[1] = (int) round(colorMapJet[idx][1] * 255.0f);
                pixel[2] = (int) round(colorMapJet[idx][0] * 255.0f);
            }

            pixel += 3;
        }
    }

    cv::imwrite(fileName.c_str(), colDepth);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::meshToVRML(std::ofstream& os, std::string textureImageFileName, float scale, float maxDispDiff, U maxDist)
{
    if (!os.is_open())
    {
        D3D_THROW_EXCEPTION("Outputstream not open.")
    }

    Grid<Eigen::Matrix<U, 4, 1> > reprojPoints(width, height);
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            reprojPoints(x,y) = unproject(x,y);

            if (maxDist > 0 && reprojPoints(x,y)(3) == 1)
            {
                U dist = (reprojPoints(x,y).topRows(3) - cam.getC()).norm();
                if (maxDist < dist)
                {
                    reprojPoints(x,y)(3) = 0;
                }
            }
        }

    os << "#VRML V2.0 utf8" << endl;
    os << "Shape {" << endl;
    os << "     appearance Appearance {" << endl;
    os << "         texture ImageTexture {" << endl;
    os << "             url [\"" << textureImageFileName << "\"]" << endl;
    os << "         }" << endl;
    os << "         material Material { " << endl;
    os << "             diffuseColor     0.5 0.5 0.5" << endl;
    os << "         }" << endl;
    os << "     }" << endl;
    os << "     geometry IndexedFaceSet {" << endl;
    os << "       coord Coordinate {" << endl;
    os << "           point [" << endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," << endl;
    os << "           ]" << endl;
    os << "       }" << endl;
    os << "       coordIndex [" << endl;
    for (unsigned int y = 0; y < height-1; y++)
        for (unsigned int x = 0; x < width-1; x++)
        {
            float dispDiff = (float) std::max(std::fabs(scale/(*this)(x,y) - scale/(*this)(x+1,y)) , std::max(std::fabs(scale/(*this)(x+1,y)-scale/(*this)(x,y+1)), std::fabs(scale/(*this)(x,y+1))- scale/(*this)(x,y)));
            if ((maxDispDiff < 0 || dispDiff < maxDispDiff) &&  reprojPoints(x,y)(3) == 1 && reprojPoints(x+1,y)(3) == 1 && reprojPoints(x,y+1)(3) == 1)
                os << "           " << y*width + x +1 << ", " << y*width + x << ", " << (y+1)*width + x << ", -1," << endl;
            dispDiff = (float) std::max(std::fabs(scale/(*this)(x+1,y+1)-scale/(*this)(x+1,y)), std::max(std::fabs(scale/(*this)(x+1,y)-scale/(*this)(x,y+1)), std::fabs(scale/(*this)(x,y+1))-scale/(*this)(x+1,y+1)));
            if ((maxDispDiff < 0 || dispDiff < maxDispDiff) && reprojPoints(x+1,y+1)(3) == 1 && reprojPoints(x+1,y)(3) == 1 && reprojPoints(x,y+1)(3) == 1)
                os << "           " << (y+1)*width + x + 1 << ", " <<  y*width + x +1 << ", " << (y+1)*width + x  << ", -1," << endl;
        }
    os << "       ]" << endl;
    os << "       texCoord TextureCoordinate {" << endl;
    os << "         point [" << endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "                 " << (float) x/(float)width << " " << height - (float)y/(float)height << ", " << endl;
    os << "         ]" << endl;
    os << "     }" << endl;
    os << "   }" << endl;
    os << "}" << endl;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::saveAsDataFile(string fileName)
{

    if (CHAR_BIT != 8)
    {
        D3D_THROW_EXCEPTION("Only platforms with 8 bit chars are supported.")
    }

    std::ofstream outStream;
    outStream.open(fileName.c_str(), std::ios::out | std::ios::binary);

    if (!outStream.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open depth map data output file.")
    }

    // file format version, might be useful at some point
    unsigned char version = 1;
    outStream.write((char*)&version, 1);

    // endianness
    unsigned char endian = is_little_endian() ? 0 : 1;
    outStream.write((char*)&endian, 1);

    // store sizes of data types written
    // first unsigned int in an unsigned char because we know that char has always size 1
    unsigned char uintSize = sizeof(unsigned int);
    outStream.write((char*)&uintSize, 1);

    // for T and U we use unsigned int
    unsigned int tSize = sizeof(T);
    outStream.write((char*)&tSize, sizeof(unsigned int));
    unsigned int uSize = sizeof(U);
    outStream.write((char*)&uSize, sizeof(unsigned int));

    // store K, R, T, Xi
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            outStream.write((char*)&(cam.getK()(i,j)), sizeof(U));

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            outStream.write((char*)&(cam.getR()(i,j)), sizeof(U));

    for (int i = 0; i < 3; i++)
        outStream.write((char*)&(cam.getT()(i)), sizeof(U));

    outStream.write((char*)&(cam.getXi()), sizeof(U));

    // now we store the size of the depth map
    outStream.write((char*)&width, sizeof(unsigned int));
    outStream.write((char*)&height, sizeof(unsigned int));
    outStream.write((char*)getDataPtr(), sizeof(T)*width*height);

    if (!outStream.good())
    {
        D3D_THROW_EXCEPTION("An error occured while writing out a depth map to a data file.")
    }

    // writing is done closing stream
    outStream.close();
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::loadFromDataFile(string fileName)
{
    if (CHAR_BIT != 8)
    {
        D3D_THROW_EXCEPTION("Only platforms with 8 bit chars are supported.")
    }

    std::ifstream inStream;
    inStream.open(fileName.c_str(), std::ios::in | std::ios::binary);

    if (!inStream.is_open())
    {
        D3D_THROW_EXCEPTION("Could not open depth map data input file.")
    }

    // read in version
    unsigned char version;
    inStream.read((char*)&version, 1);
    if (version != 1)
    {
        D3D_THROW_EXCEPTION("Only version 1 is supported.")
    }

    // read in endian
    unsigned char endian;
    inStream.read((char*)&endian, 1);

    unsigned char currentEndian = is_little_endian() ? 0: 1;
    if (endian != currentEndian)
    {
        D3D_THROW_EXCEPTION("Current platform does not have the same endian as the depht map data file.")
    }

    // read in the size of an unsigned int from file
    unsigned char uintSize;
    inStream.read((char*)&uintSize, 1);

    // check if current plattform has the same unsigned int size
    if (uintSize != sizeof (unsigned int))
    {
        D3D_THROW_EXCEPTION("Current platform does not have the same unsigned int size as the one the file was written with.")
    }

    unsigned int tSize;
    unsigned int uSize;
    inStream.read((char*)&tSize, sizeof(unsigned int));
    inStream.read((char*)&uSize, sizeof(unsigned int));
    if (tSize != sizeof(T) || uSize != sizeof(U))
    {
        D3D_THROW_EXCEPTION("Sizes of the datatypes for sparse and dense geometry do not match.")
    }

    // read K, R, T, C, Xi
    Eigen::Matrix<U, 3, 3> K, R;
    Eigen::Matrix<U, 3, 1> Tr;
    U Xi;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            inStream.read((char*)&(K(i,j)), sizeof(U));

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            inStream.read((char*)&(R(i,j)), sizeof(U));

    for (int i = 0; i < 3; i++)
        inStream.read((char*)&(Tr(i)), sizeof(U));

    inStream.read((char*)&(Xi), sizeof(U));

    cam.setKRTXi(K, R, Tr, Xi);

    // read the size of the depth map
    inStream.read((char*)&width, sizeof(unsigned int));
    inStream.read((char*)&height, sizeof(unsigned int));

    // set the shared array to the right size
    T* depths = new T[width*height];
    this->depths.reset(depths);

    // read in the depth data
    inStream.read((char*)getDataPtr(), sizeof(T)*width*height);

    // check if we have read the depth map correctly
    if (!inStream.good())
    {
        D3D_THROW_EXCEPTION("Error reading in the depth map data file.")
    }

    // colse the stream
    inStream.close();
}





// instantiate
template class FishEyeDepthMap<float, float>;
template class FishEyeDepthMap<float, double>;
template class FishEyeDepthMap<double, float>;
template class FishEyeDepthMap<double, double>;
