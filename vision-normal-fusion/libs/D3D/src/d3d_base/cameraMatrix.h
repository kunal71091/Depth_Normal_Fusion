#ifndef CAMERAMATRIX_H
#define CAMERAMATRIX_H

#include <Eigen/Dense>
#include <string>

namespace D3D
{
    template <typename NumericType>
    class CameraMatrix
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CameraMatrix();
        CameraMatrix(const Eigen::Matrix<NumericType, 3, 4>& P);
        CameraMatrix(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T);

        // copy consturctor and assignment
        CameraMatrix(const CameraMatrix<NumericType>& otherCameraMatrix);
        CameraMatrix& operator=(const CameraMatrix<NumericType>& otherCameraMatrix);

        void scaleK(NumericType scale_x, NumericType scale_y);

        void setKRT(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T);
        void setP(const Eigen::Matrix<NumericType, 3, 4>& P);
        void setRT(const Eigen::Matrix<NumericType, 3, 4>& RT);

        const Eigen::Matrix<NumericType, 3, 3>& getK() const;
        const Eigen::Matrix<NumericType, 3, 3>& getR() const;
        const Eigen::Matrix<NumericType, 3, 1>& getT() const;
        const Eigen::Matrix<NumericType, 3, 4>& getP() const;
        const Eigen::Matrix<NumericType, 3, 1>& getC() const;
        const Eigen::Matrix<NumericType, 4, 4>& getCam2Global() const;

        Eigen::Matrix<NumericType, 4, 1> unprojectPoint(NumericType x, NumericType y, NumericType depth) const;

        Eigen::Matrix<NumericType, 4, 1> localPoint2GlobalPoint(Eigen::Matrix<NumericType, 4, 1>& localPoint) const;

        void loadFromDepthMapDataFile(std::string fileName);

    private:
        Eigen::Matrix<NumericType, 3, 3> K;
        Eigen::Matrix<NumericType, 3, 3> R;
        Eigen::Matrix<NumericType, 3, 1> T;

        void recomputeStoredValues();

        // some prestored values to return
        Eigen::Matrix<NumericType, 3, 4> P;
        Eigen::Matrix<NumericType, 4, 4> cam2Global;
        Eigen::Matrix<NumericType, 3, 1> C;
    };
}


#endif // CAMERAMATRIX_H
