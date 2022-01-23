#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/opencv.hpp>

void draw_kps(cv::Mat &image, std::vector<cv::Point2f> p0,
              std::vector<cv::Point2f> p1) {
  for (size_t idx = 0; idx < p0.size(); idx++) {
    cv::circle(image, p0[idx], 1, cv::Scalar(0, 0, 225), cv::FILLED,
               cv::LINE_8);
    cv::circle(image, p1[idx], 1, cv::Scalar(255, 0, 0), cv::FILLED,
               cv::LINE_8);
  }
}

#endif // UTILS_H_
