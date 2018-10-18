#include "grid.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace D3D
{

    template<typename Elem>
    void saveGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, const char* fileName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));

        double min, max;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(sliceMat, &min, &max, &minLoc, &maxLoc);

        cv::Mat_<Elem> sliceMat2 = sliceMat - min;
        cv::imwrite(fileName, sliceMat2/(max-min)*255);
    }

    template<typename Elem>
    void saveGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, Elem minVal, Elem maxVal, const char* fileName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));
        cv::Mat_<Elem> sliceMat2 = sliceMat - minVal;
        cv::imwrite(fileName, sliceMat2/(maxVal-minVal)*255);
    }

    template<typename Elem>
    void displayGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, long time, const char* windowName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));

        double min, max;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(sliceMat, &min, &max, &minLoc, &maxLoc);

        cv::Mat_<Elem> sliceMat2 = sliceMat - min;
        cv::imshow(windowName, sliceMat2/(max-min));
        cv::waitKey(time);
    }

    template<typename Elem>
    void displayGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, Elem minVal, Elem maxVal,  long time, const char* windowName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));
        cv::Mat_<Elem> sliceMat2 = sliceMat - minVal;

        cv::imshow(windowName, sliceMat2/(maxVal-minVal));
        cv::waitKey(time);
    }

    template void saveGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, const char* fileName);
    template void saveGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, double minVal, double maxVal, const char* fileName);

    template void saveGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, const char* fileName);
    template void saveGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, float minVal, float maxVal, const char* fileName);

    template void displayGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, long time, const char* windowName);
    template void displayGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, double minVal, double maxVal, long time, const char* windowName);

    template void displayGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, long time, const char* windowName);
    template void displayGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, float minVal, float maxVal, long time, const char* windowName);
}
