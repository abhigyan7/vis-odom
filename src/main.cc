#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include <stdint.h>

void triangulate_points(cv::Mat img_1, cv::Mat img_2, double focal, cv::Point2d pp)
{
    std::vector<cv::Point2f> p0, p1;

    cv::goodFeaturesToTrack(img_1, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

    std::vector<uchar> status;
    std::vector<float> error;

    std::vector<cv::Point2f> points_2;
    std::vector<cv::Point2f> points_1;
    cv::goodFeaturesToTrack(img_1, points_1, 2000, 0.01, 10, cv::Mat(), 3, 3, 0, 0.4);
    //std::cout << points_1.size() << std::endl;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
    cv::calcOpticalFlowPyrLK(img_1, img_2, points_1, points_2, status, error, cv::Size(15,15), 2, criteria);

    std::cout << "calculated optical flow" << std::endl;

    size_t i, k;
    for (i = k = 0; i < points_1.size(); i++)
    {
        if (! status[i])
            continue;
        points_1[k++] = points_1[i];
        points_2[k] = points_2[i];
    }

    points_1.resize(k);
    points_2.resize(k);

    cv::Mat essential_matrix = cv::findEssentialMat(points_1, points_2, focal, pp, cv::RANSAC, 0.999, 1.0, 1000);

    std::cout << "Essential Matrix: " << essential_matrix << std::endl;
    // recoverPose
    // triangulatePoints

}

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: main <video-file>.mp4\n");
        return -1;
    }
    cv::VideoCapture vidcap;
    vidcap.open(argv[1]);
    polyscope::init();
    std::vector<glm::vec3> points;

    //// generate points
    //for (size_t i = 0; i < 3000; i++) {
    //  points.push_back(
    //      glm::vec3{polyscope::randomUnit() - .5,
    //                polyscope::randomUnit() - .5,
    //                polyscope::randomUnit() - .5});
    //}

    cv::Mat image_1c, image_2c;
    cv::Mat image_1, image_2;
    vidcap >> image_1c;
    cv::cvtColor(image_1c, image_1, cv::COLOR_BGR2GRAY);

    vidcap >> image_2c;
    cv::cvtColor(image_2c, image_2, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> points_1;
    cv::goodFeaturesToTrack(image_1, points_1, 2000, 0.01, 10, cv::Mat(), 3, 3, 0, 0.4);
    std::cout << "Found these points: " << points_1 << std::endl;
    //void triangulate_points(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> points_1, double focal, cv::Point2d pp)
    cv::Point2f pp;
    pp.x = 0.0;
    pp.y = 0.0;
    triangulate_points(image_1, image_2, 700.0, pp);
    // visualize!

    polyscope::PointCloud* psCloud = polyscope::registerPointCloud("really great points", points);

    // set some options
    psCloud->setPointRadius(0.02);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
    polyscope::show();
    //cv::Mat image;
    //image = cv::imread( argv[1], 1 );

    //if ( !image.data )
    //{
    //    printf("No image data \n");
    //    return -1;
    //}

    //cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    //cv::imshow("Display Image", image);
    //cv::waitKey(0);
    return 0;
}
