// #define ENABLE_ESTIMATOR
#define DEBUG_USE_SEGMENTS_MAP

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <numeric>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include "rgb_cloud_segmentation/dist_transform_segmenter.h"
#include "dist_transform_segmenter.cpp"


using PointType = pcl::PointXYZRGB;
using PointNoColorType = pcl::PointXYZ;
using PointWithLabelType = pcl::PointXYZL;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudWithLabel = pcl::PointCloud<PointWithLabelType>;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointCloudNoColorPtr = pcl::PointCloud<PointNoColorType>::Ptr;
using PointCloudWithLabelPtr = PointCloudWithLabel::Ptr;

PointInTPtr scene_cloud;
std::vector<std::vector<cv::Point>> image_segments;
cv::Mat segments_map;
std::vector<PointCloudNoColorPtr> segment_clouds;
int image_width, image_height;

// Algorithm parameters
int dist_transform_mask_size(5);
bool visualize(true);

void convertPCLCloud2Image(const PointInTPtr &cloud, cv::Mat_<cv::Vec3b> &image)
{
    image_width = cloud->width;
    image_height = cloud->height;
    int position = 0;

    //    std::cout << "Point cloud dimensions: " << image_height << " x " << image_width << "\n";

    image = cv::Mat_<cv::Vec3b>(image_height, image_width);

    for (int row = 0; row < image_height; row++)
    {
        for (int col = 0; col < image_width; col++)
        {
            cv::Vec3b &cvp = image.at<cv::Vec3b>(row, col);
            position = row * image_width + col;
            const PointType &pt = cloud->points[position];

            cvp[0] = pt.b;
            cvp[1] = pt.g;
            cvp[2] = pt.r;
        }
    }

    //    std::cout << "Point cloud was converted to RGB image successfully\n";
}

/**
 * Segment point cloud using areas obtained from RGB image
 */
void segmentCloud()
{
    PointCloudWithLabelPtr labeled_cloud(new PointCloudWithLabel);
    labeled_cloud->points.resize(scene_cloud->points.size());
    labeled_cloud->is_dense = false;
    labeled_cloud->width = static_cast<int>(labeled_cloud->points.size()); // image_width;
    labeled_cloud->height = 1;                                             // image_height;

    for (int j = 0; j < segments_map.rows; j++) // image_height
    {
        for (int k = 0; k < segments_map.cols; k++) // image_width
        {
            int pos = j * image_width + k;

            PointWithLabelType labeled_p;
            labeled_p.x = scene_cloud->points[pos].x;
            labeled_p.y = scene_cloud->points[pos].y;
            labeled_p.z = scene_cloud->points[pos].z;

            if (segments_map.at<uchar>(j, k) != 0)
            {
                labeled_p.label = segments_map.at<uchar>(j, k);
            }

            labeled_cloud->points[pos] = labeled_p;
        }
    }

    std::string labeled_cloud_pcd = "labeled_cloud.pcd";

    pcl::io::savePCDFileASCII(labeled_cloud_pcd.c_str(), *labeled_cloud);

    std::cout << "\n";
}

void showHelp(char *filename)
{
    PCL_INFO("Usage: %s <scene_pcd> [options]\n", filename);
    PCL_INFO("* where options are:\n");
    PCL_INFO("--mask_size <mask_size>:\tdistance transform mask size (0, 3 or 5)\n");
    PCL_INFO("-vis:\t\t\t\tvisualize the results\n");
}

void parseCommandLine(int argc, char **argv)
{
    if (argc < 2)
    {
        PCL_ERROR("Scene PCD file missing\n");
        showHelp(argv[0]);
        exit(-1);
    }

    pcl::console::parse_argument(argc, argv, "--mask_size", dist_transform_mask_size);

    visualize = pcl::console::find_switch(argc, argv, "-vis");
}

int main(int argc, char **argv)
{
    parseCommandLine(argc, argv);

    PCL_INFO("Dist transform mask size: %d\n", dist_transform_mask_size);

    PCL_INFO("Visualize: %s\n", visualize ? "true" : "false");

    scene_cloud.reset(new PointCloud());
    pcl::io::loadPCDFile(argv[1], *scene_cloud);

    PCL_INFO("scene cloud has %d points\n", scene_cloud->points.size());

    cv::Mat_<cv::Vec3b> cloud_img;
    convertPCLCloud2Image(scene_cloud, cloud_img);

    Mat img = cloud_img;

    cv::imwrite("scene_rgb.jpg", img);

    DistTransformSegmenter segmenter(dist_transform_mask_size, visualize);
    segmenter.loadImage(img);

    segmenter.segment();
    segmenter.findAreas();

    //    image_segments = segmenter.getAreas();

    segments_map = segmenter.getSegmentsMap();

    // TODO Add logic for segmenting cloud
    segmentCloud();

    return 0;
}
