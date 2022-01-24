#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/opencv.hpp>

void draw_camera(Eigen::Vector3d origin, Eigen::Vector3d direction,
                 double focal_length, double aspect_ratio, double scale) {

  scale = 0.2 * ((origin - direction).norm());

  double dx = scale / 3.0;
  double dy = scale / 3.0;
  glBegin(GL_LINES);

  glVertex3f(origin(0), origin(1), origin(2));
  glVertex3f(origin(0) + direction(0) + dx,
             origin(1) + scale * direction(1) + dy,
             origin(2) + scale * direction(2));

  glVertex3f(origin(0), origin(1), origin(2));
  glVertex3f(origin(0) + scale * direction(0) - dx,
             origin(1) + direction(1) + dy, origin(2) + scale * direction(2));

  glVertex3f(origin(0), origin(1), origin(2));
  glVertex3f(origin(0) + scale * direction(0) - dx,
             origin(1) + direction(1) - dy, origin(2) + scale * direction(2));

  glVertex3f(origin(0), origin(1), origin(2));
  glVertex3f(origin(0) + scale * direction(0) + dx,
             origin(1) + direction(1) - dy, origin(2) + scale * direction(2));

  glVertex3f(origin(0) + scale * direction(0) + dx,
             origin(1) + direction(1) + dy, origin(2) + scale * direction(2));
  glVertex3f(origin(0) + scale * direction(0) - dx,
             origin(1) + direction(1) + dy, origin(2) + scale * direction(2));

  glVertex3f(origin(0) + scale * direction(0) + dx,
             origin(1) + direction(1) + dy, origin(2) + scale * direction(2));
  glVertex3f(origin(0) + scale * direction(0) + dx,
             origin(1) + direction(1) - dy, origin(2) + scale * direction(2));

  glVertex3f(origin(0) + scale * direction(0) + dx,
             origin(1) + direction(1) - dy, origin(2) + scale * direction(2));
  glVertex3f(origin(0) + scale * direction(0) - dx,
             origin(1) + direction(1) - dy, origin(2) + scale * direction(2));

  glVertex3f(origin(0) + scale * direction(0) - dx,
             origin(1) + direction(1) + dy, origin(2) + scale * direction(2));
  glVertex3f(origin(0) + scale * direction(0) - dx,
             origin(1) + direction(1) - dy, origin(2) + scale * direction(2));

  glEnd();
}

void draw_cameras_from_trajectory(std::vector<Eigen::Vector3d> traj_points,
                                  double f, double a, double s) {
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    draw_camera(traj_points[i], traj_points[i + 1], f, a, s);
  }
}

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
