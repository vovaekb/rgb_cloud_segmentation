#include "rgb_cloud_segmentation/dist_transform_segmenter.h"

DistTransformSegmenter::DistTransformSegmenter(int dist_transform_mask_size, bool visualize):
    dist_transform_mask_size_ (dist_transform_mask_size),
    visualize_ (visualize)
{
    min_size_ = 30;
}

void DistTransformSegmenter::loadImage(cv::Mat &image)
{
    src_ = image;
}

void DistTransformSegmenter::segment()
{
    // Check if everything was fine
    if (!src_.data)
        return;
    // Show source image
    cv::imshow("Source Image", src_);
    // Change the background from white to black, since that will help later to extract
    // better results during the use of Distance Transform
    for( int x = 0; x < src_.rows; x++ ) {
      for( int y = 0; y < src_.cols; y++ ) {
          if ( src_.at<cv::Vec3b>(x, y) == cv::Vec3b(255,255,255) ) {
            src_.at<cv::Vec3b>(x, y)[0] = 0;
            src_.at<cv::Vec3b>(x, y)[1] = 0;
            src_.at<cv::Vec3b>(x, y)[2] = 0;
          }
        }
    }
    // Show output image
//    cv::imshow("Black Background Image", src_);
    // Create a kernel that we will use for accuting/sharpening our image
    cv::Mat kernel = (cv::Mat_<float>(3,3) <<
            1,  1, 1,
            1, -7.19, 1, // -7.2 // -8
            1,  1, 1); // an approximation of second derivative, a quite strong kernel
    // do the laplacian filtering as it is
    // well, we need to convert everything in something more deeper then CV_8U
    // because the kernel has some negative values,
    // and we can expect in general to have a Laplacian image with negative values
    // BUT a 8bits unsigned int (the one we are working with) can contain values from 0 to 255
    // so the possible negative number will be truncated
    cv::Mat imgLaplacian;
    cv::Mat sharp = src_; // copy source image to another temporary one
    cv::filter2D(sharp, imgLaplacian, CV_32F, kernel);
    src_.convertTo(sharp, CV_32F);
    cv::Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    // cv::imshow( "Laplace Filtered Image", imgLaplacian );
//    cv::imshow( "New Sharped Image", imgResult );
//    cv::waitKey(10);
    src_ = imgResult; // copy back
    // Create binary image from source image
    cv::Mat bw;
    cv::cvtColor(src_, bw, CV_BGR2GRAY);
    cv::threshold(bw, bw, 100, 255, CV_THRESH_BINARY | CV_THRESH_OTSU); // 40 // 20
    cv::imshow("Binary Image", bw);
//    cv::imwrite("scene_binary.jpg", bw);
//    cv::waitKey(10);
    // Perform the distance transform algorithm
    cv::Mat dist;
    cv::distanceTransform(bw, dist, CV_DIST_L2, dist_transform_mask_size_); // available values: 0, 3, 5 // 3 - default
    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    cv::normalize(dist, dist, 0, 1., NORM_MINMAX);
//    cv::imshow("Distance Transform Image", dist);
//    cv::waitKey(10);
//    cv::imwrite("scene_dist_transform.jpg", dist);
    // Threshold to obtain the peaks
    // This will be the markers for the foreground objects
    cv::threshold(dist, dist, .4, 1., CV_THRESH_BINARY); // .4
    // Dilate a bit the dist image
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
    cv::dilate(dist, dist, kernel1);
//    cv::imshow("Peaks", dist);
//    cv::waitKey(10);
//    cv::imwrite("scene_peaks.jpg", dist);
    // Create the CV_8U version of the distance image
    // It is needed for findContours()
    cv::Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // Find total markers
    cv::findContours(dist_8u, contours_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::cout << "Contours number: " << contours_.size() << "\n";
}

void DistTransformSegmenter::findAreas()
{
//    cv::Mat drawing = cv::Mat::zeros( src_.size(), CV_8UC3 );
    result_ = cv::Mat::zeros( src_.size(), CV_8UC3 );

    RNG rng(12345);
    for( int i = 0; i < contours_.size(); i++ )
    {
        Mat area_img = Mat::zeros( src_.size(), CV_8UC3 );
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        drawContours( drawing, contours_, i, color, CV_FILLED, 8, CV_RETR_EXTERNAL, 0, cv::Point() ); // hierarchy

        drawContours( area_img, contours_, i, color, CV_FILLED, 8, CV_RETR_EXTERNAL, 0, Point() );

        std::vector<cv::Point> area_points;

//        std::cout << "Contour " << i << " has area " << contour_area << "\n";

        for(size_t j = 0; j < area_img.rows; j++)
        {
            for(size_t k = 0; k < area_img.cols; k++)
            {
                if(area_img.at<uchar>(j, k) != 0)
                {
                    cv::Point p;
                    p.x = k; p.y = j;
                    area_points.push_back(p);
                }
            }
        }

        // Reject too small areas
        if(area_points.size() < min_size_) continue;

        result_ = result_ + area_img;

        areas_.push_back(area_points);

//        std::cout << "Area " << i << " has " << area_points.size() << " points\n\n";
    }

//    std::cout << "Areas number: " << areas_.size() << "\n";

    if(visualize_)
    {
        cv::imshow( "Contours", result_ );
        cv::imwrite("areas.jpg", result_);
        cv::waitKey(0);
    }
}

cv::Mat DistTransformSegmenter::getSegmentsMap()
{
    return result_;
}

std::vector<std::vector<cv::Point> > DistTransformSegmenter::getAreas()
{
    return areas_;
}
