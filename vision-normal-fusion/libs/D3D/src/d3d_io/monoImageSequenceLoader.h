#ifndef MONOIMAGESEQUENCELOADER_H
#define MONOIMAGESEQUENCELOADER_H

#include <d3d_base/cameraMatrix.h>

#include <opencv2/core/core.hpp>

using cv::Mat;

namespace D3D {

    class MonoImageSequenceLoader
    {
    public:
        virtual bool atEOS() = 0;
        virtual void moveNext() = 0;
        virtual Mat getImage() = 0;
        virtual CameraMatrix<double> getCam() = 0;

        virtual ~MonoImageSequenceLoader() {}
    };

}

#endif // MONOIMAGESEQUENCELOADER_H
