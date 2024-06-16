#ifndef DIST_TRANSFORM_SEGMENTER_H
#define DIST_TRANSFORM_SEGMENTER_H

#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

/**
 * Class for performing Distance Transform segmentation in OpenCV
 */
class DistTransformSegmenter
{
public:
    using points_vector = std::vector<cv::Point>;
    DistTransformSegmenter(int dist_transform_mask_size, bool visualize);

    void loadImage(cv::Mat &image);

    void segment();

    void findAreas();

    std::vector<points_vector> getAreas();

    cv::Mat getSegmentsMap();

protected:
    // parameters
    int min_size_; // min size for contour allowed
    int dist_transform_mask_size_;
    bool visualize_;

    // data
    cv::Mat src_;
    cv::Mat result_;
    std::vector<points_vector> contours_;
    std::vector<points_vector> areas_;
};

#endif // DIST_TRANSFORM_SEGMENTER_H
