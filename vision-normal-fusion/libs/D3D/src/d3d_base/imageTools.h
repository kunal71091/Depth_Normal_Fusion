#ifndef IMAGETOOLS_H
#define IMAGETOOLS_H

#include <opencv2/core/core.hpp>
#include <string>

namespace D3D
{
    void displayImageScaled(cv::Mat img, float scale, std::string windowName, int time = 0);
    void findImageSegmentsBGRAIgnoringA(const cv::Mat& image, cv::Mat& segmentIDs);
    void drawSegmentationBoundariesOnImage(const cv::Mat& segmentIDs, cv::Mat& img, unsigned char r = 255, unsigned char g = 0, unsigned char b = 0);

    void rotateImageLeft(const cv::Mat& srcImg, cv::Mat& dstImg);
    void rotateImageRight(const cv::Mat& srcImg, cv::Mat& dstImg);

    template <typename T>
    void boxFilterImage(const cv::Mat_<T> &img, int radius_x, int radius_y, cv::Mat_<T> &filteredImg);
}
#endif // IMAGETOOLS_H
