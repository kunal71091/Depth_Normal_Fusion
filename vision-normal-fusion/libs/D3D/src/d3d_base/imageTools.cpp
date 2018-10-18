#include "imageTools.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <list>
#include <d3d_base/exception.h>

namespace D3D
{

void displayImageScaled(cv::Mat img, float scale, std::string windowName, int time)
{
    cv::Mat scaledImage;
    if (scale < 1)
        resize(img, scaledImage, cv::Size(0,0), scale, scale, cv::INTER_AREA);
    else
        resize(img, scaledImage, cv::Size(0,0) , scale, scale, cv::INTER_LINEAR);

    cv::imshow(windowName.c_str(), scaledImage);
    cv::waitKey(time);
}

void findImageSegmentsBGRAIgnoringA(const cv::Mat& image, cv::Mat& segmentIDs)
{
    segmentIDs = cv::Mat(image.size(), CV_32SC1);
    segmentIDs = -1;

    int nextID = 0;
    const int rows = segmentIDs.rows;
    const int cols = segmentIDs.cols;
    for (int y = 0; y < rows; y++)
        for (int x = 0; x < cols; x++)
        {
            if (segmentIDs.at<int>(y,x) == -1)
            {
                std::list<std::pair<int, int> > toProcess;
                toProcess.push_back(std::make_pair<int, int>(y,x));
                unsigned char b = image.at<unsigned char>(y,4*x);
                unsigned char g = image.at<unsigned char>(y,4*x + 1);
                unsigned char r = image.at<unsigned char>(y,4*x + 2);
                while (!toProcess.empty())
                {
                    std::pair<int, int> curr = toProcess.back();
                    toProcess.pop_back();

                    if ((segmentIDs.at<int>(curr.first, curr.second) == -1) &&
                            (image.at<unsigned char>(curr.first, 4*curr.second) == b) &&
                            (image.at<unsigned char>(curr.first, 4*curr.second+1) == g) &&
                            (image.at<unsigned char>(curr.first, 4*curr.second+2) == r))
                    {
                        segmentIDs.at<int>(curr.first, curr.second) = nextID;


                        for (int wy=-1; wy <= 1; wy++)
                            for (int wx=-1; wx <= 1; wx++)
                            {
                                if (!(wx == 0 && wy == 0))
                                {
                                    std::pair<int, int> curr0 = std::make_pair(curr.first + wy, curr.second + wx);
                                    if (curr0.first >= 0 && curr0.first < rows && curr0.second >= 0 && curr0.second < cols)
                                    {
                                        if (segmentIDs.at<int>(curr0.first, curr0.second) == -1)
                                        {
                                            toProcess.push_back(curr0);
                                        }
                                    }
                                }
                            }
                    }
                }
                nextID++;
            }
        }
}

void drawSegmentationBoundariesOnImage(const cv::Mat& segmentIDs, cv::Mat& img, unsigned char r, unsigned char g, unsigned char b)
{
    if (img.rows != segmentIDs.rows || img.cols != segmentIDs.cols)
    {
        D3D_THROW_EXCEPTION("Sizes of img and segmentIDs do not match.")
    }

    if (img.type() != CV_8UC3)
    {
        D3D_THROW_EXCEPTION("Only images of type CV_8UC3 are supported.")
    }

    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
        {
            int id = segmentIDs.at<int>(y,x);
            bool boundary = false;
            for (int wy = -1; wy <= 1 && !boundary; wy++)
                for (int wx = -1; wx <= 1 && !boundary; wx++)
                {
                    const int y0 = y+wy;
                    const int x0 = x+wx;

                    if (x0 >= 0 && x0 < img.cols && y0 >= 0 && y0 < img.rows)
                    if (segmentIDs.at<int>(y0,x0) != id)
                        boundary = true;
                }

            if (boundary)
            {
                img.at<unsigned char>(y,3*x) = b;
                img.at<unsigned char>(y,3*x+1) = g;
                img.at<unsigned char>(y,3*x+2) = r;
            }
        }
}

// only for debugging not intended to be used
template <typename T>
void boxFilterImage(const cv::Mat_<T> &img, int radius_x, int radius_y, cv::Mat_<T> &filteredImg)
{
    filteredImg = cv::Mat_<T>::zeros(img.size());

    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            long windowVal = 0;
            for (int yw = y - radius_y; yw <= y + radius_y; yw++)
            {
                int yw0 = std::max(0, std::min(yw, img.rows-1));
                for (int xw = x - radius_x; xw <= x + radius_x; xw++)
                {
                    int xw0 = std::max(0, std::min(xw, img.cols-1));

                    windowVal += img(yw0,xw0);
                }
            }
            filteredImg(y,x) = windowVal/((2*radius_x+1)*(2*radius_y+1));
        }
    }
}

template void boxFilterImage<unsigned char>(const cv::Mat_<unsigned char>& img, int radius_x, int radius_y, cv::Mat_<unsigned char>& filteredImg);

void rotateImageLeft(const cv::Mat& srcImg, cv::Mat& dstImg)
{
    int w = srcImg.cols;
    int h = srcImg.rows;
    size_t elemSize = srcImg.elemSize();
    dstImg.create(srcImg.cols, srcImg.rows, srcImg.type());
    for(int y=0; y<h; y++)
        for(int x=0; x<w; x++)
        {
            unsigned char* dstPtr = &(dstImg.at<unsigned char>(w-1-x,y*elemSize));
            const unsigned char* srcPtr = &(srcImg.at<unsigned char>(y,x*elemSize));
            memcpy(dstPtr, srcPtr, elemSize);
        }
}

void rotateImageRight(const cv::Mat& srcImg, cv::Mat& dstImg)
{
    int w = srcImg.cols;
    int h = srcImg.rows;
    size_t elemSize = srcImg.elemSize();
    dstImg = cv::Mat(srcImg.cols, srcImg.rows, srcImg.type());
    for(int y=0; y<h; y++)
        for(int x=0; x<w; x++)
        {
            char* dstPtr = &(dstImg.at<char>(x,(h-1-y)*elemSize));
            const char* srcPtr = &(srcImg.at<char>(y,x*elemSize));
            memcpy(dstPtr, srcPtr, elemSize);
        }
}

}
