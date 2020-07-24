#ifndef IMAGE_UTILS_HPP_
#define IMAGE_UTILS_HPP_

#include <iostream>
#include <vector>
#include <typeinfo>


#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>


class image_utils {
  public:
    image_utils (){};

    bool removeSpeckles(cv::Mat& ioImage, const double iMaxDisparityDiff,
                        const int iMaxBlobSize);
    bool removeSmall(cv::Mat& ioImage, const uint16_t iValueThresh,
                     const int iSizeThresh);

    float computeIntensity(unsigned char * rgb, int row, int col, int width);

    void filterLowTexture(unsigned short * disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold, bool removeHorizontalEdges);

    void sobelEdgeFilter(unsigned short * disparity, unsigned char * rgb, int width, int height, int windowSize, double threshold, bool removeHorizontalEdges);

  private:
};

#endif
