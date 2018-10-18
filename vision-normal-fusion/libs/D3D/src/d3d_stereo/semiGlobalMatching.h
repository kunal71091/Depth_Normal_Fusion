#ifndef SEMIGLOBALMATCHING_H
#define SEMIGLOBALMATCHING_H

#include <opencv2/core/core.hpp>
#include <d3d_base/grid.h>
#include <d3d_base/depthMap.h>
#include <d3d_base/cameraMatrix.h>
#include <Eigen/Dense>

namespace D3D
{

class SemiGlobalMatching
{
public:

    SemiGlobalMatching();
    SemiGlobalMatching(int _P1, int _P2, bool _fullDP = true );
    void setSmoothness(int P1, int P2);

    void matchFromPlaneSweep(const Grid<float>& costVolume, const Grid<Eigen::Vector4d>& planes, const CameraMatrix<double>& cam, float costThresh = 1e5);
    void matchFromPlaneSweepWtihImage(const Grid<float>& costVolume, const Grid<Eigen::Vector4d>& planes, const CameraMatrix<double>& cam, cv::Mat image, float P1F, float P2F, float k);

    void regularizeCostVolumeWithImage(const Grid<float>& costVolume, cv::Mat image, float P1F, float P2F, float k);

    void enableOutputDepthMap(bool enabled = true);
    void enableOutputUniquenessRatios(bool enabled = true);
    void enableOutputCostsEnabled(bool enabled = true);
    void enableOutputLabels(bool enabled = true);

    DepthMap<float, double> getDepthMap();
    Grid<float> getCosts();
    Grid<float> getUniquenessRatios();
    Grid<int> getLabels();

private:
    enum { NR = 16, NR2 = NR/2 };
    enum { DISP_SHIFT=4, DISP_SCALE = (1<<DISP_SHIFT) };

    typedef unsigned char PixType;
    typedef short CostType;
    typedef short DispType;
//    int minDisparity;
//    int numberOfDisparities;
//    int SADWindowSize;
    int P1;
    int P2;
//    int disp12MaxDiff;
//    int preFilterCap;
//    int uniquenessRatio;
//    int speckleWindowSize;
//    int speckleRange;
    bool fullDP;

    Grid<float> costVolume;

    bool outputDepthMapEnabled;
    bool outputUniquenessRatiosEnabled;
    bool outputCostsEnabled;
    bool subPixelEnabled;

    bool outputLabelsEnabled;

    DepthMap<float, double> result;
    Grid<float> uniquenessRatios;
    Grid<float> costs;

    Grid<int> labels;
    Grid<float> offsets;


protected:
    cv::Mat buffer;

private:
    void computeDisparitySGBM( const Grid<float>& costVolume,
                                     cv::Mat& disp1, const SemiGlobalMatching& params,
                                     cv::Mat& buffer );
};
}

#endif // SEMIGLOBALMATCHING_H
