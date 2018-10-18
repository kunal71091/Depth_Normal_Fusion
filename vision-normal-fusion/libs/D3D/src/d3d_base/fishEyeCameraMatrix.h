#ifndef FISHEYECAMERAMATRIX_H
#define FISHEYECAMERAMATRIX_H

#include <Eigen/Dense>
#include <string>
#include "cameraMatrix.h"

namespace D3D
{
    template <typename NumericType>
    class FishEyeCameraMatrix
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FishEyeCameraMatrix();
        FishEyeCameraMatrix(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T, NumericType xi);

        const NumericType& getXi() const;
        void scaleK(NumericType scale_x, NumericType scale_y);
        void setKRTXi(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T, NumericType xi);

        const Eigen::Matrix<NumericType, 3, 3>& getK() const;
        const Eigen::Matrix<NumericType, 3, 3>& getR() const;
        const Eigen::Matrix<NumericType, 3, 1>& getT() const;
        const Eigen::Matrix<NumericType, 3, 1>& getC() const;

        Eigen::Matrix<NumericType, 4, 1> unprojectPoint(NumericType x, NumericType y, NumericType depth) const;
        Eigen::Matrix<NumericType, 2, 1> projectPoint(NumericType x, NumericType y, NumericType z) const;

    private:
        CameraMatrix<NumericType> cam;
        NumericType xi;
    };
}

#endif // FISHEYECAMERAMATRIX_H
