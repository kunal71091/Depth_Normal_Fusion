#ifndef STEREOTOOLS_H
#define STEREOTOOLS_H

#include <d3d_base/grid.h>

#include <opencv2/core/core.hpp>

using cv::Mat;

namespace D3D
{
    Grid<double> computeAD(Mat img1, Mat img2, int minDisp, int maxDisp);
    Grid<float> computeBTLeft(cv::Mat imgL, cv::Mat imgR, int minDisp, int maxDisp, int windowR = 1);
    Grid<float> computeCensusMatchLeft(cv::Mat imgL, cv::Mat imgR, int minDisp, int maxDisp, int windowR = 1);
    Grid<float> computeSobelMatchLeft(cv::Mat imgL, cv::Mat imgR, int minDisp, int maxDisp, int windowR = 1);
}


#endif // STEREOTOOLS_H
