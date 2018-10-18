#ifndef VOLUMETRICFUSIONTOOLS_H
#define VOLUMETRICFUSIONTOOLS_H

#include <d3d_base/grid.h>
#include <Eigen/Dense>
#include <string>

namespace D3D
{
template <typename NumericType>
void saveVolumeAsVRMLMesh(const Grid<NumericType>& volume, NumericType isoValue,
                          const Eigen::Vector3f& minCorner, const Eigen::Vector3f& size,
                          const Eigen::Matrix4f& boxToGlobal, const Eigen::Vector3f& color,
                          std::string fileName, bool displayAxis = false,float transparency = 0.0);

template <typename NumericType>
void saveVolumeAsVRMLMeshAndTriangleFile(const Grid<NumericType>& volume, NumericType isoValue,
                                         const Eigen::Vector3f& minCorner, const Eigen::Vector3f& size,
                                         const Eigen::Matrix4f& boxToGlobal, const Eigen::Vector3f& color,
                                         std::string vrmlFileName, std::string triangleFileName, bool displayAxis = false);
}

#endif // VOLUMETRICFUSIONTOOLS_H
