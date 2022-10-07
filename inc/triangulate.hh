#ifndef TRIANGULATE_H_
#define TRIANGULATE_H_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <stdint.h>

int filter_using_mask(std::vector<cv::Point2f> &in_vector,
                      std::vector<uchar> mask);

int triangulate_points(std::vector<cv::Point2f> &points_1,
                       std::vector<cv::Point2f> &points_2, double focal,
                       cv::Point2d pp, cv::Mat &R, cv::Mat &t,
                       cv::Mat &world_points);

#endif // TRIANGULATE_H_
