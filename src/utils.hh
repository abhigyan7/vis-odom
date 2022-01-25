#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/opencv.hpp>

void mat_R_t_from_R_and_t(Eigen::Vector3d t, Eigen::Matrix3d R,
                          Eigen::Matrix4d &R_t) {
  R_t.block(0, 0, 3, 3) = R;
  R_t.block(0, 3, 3, 1) = t;
}

void draw_camera(Eigen::Vector3d cam_1_t, Eigen::Vector3d cam_2_t,
                 Eigen::Matrix3d cam_1_R, Eigen::Matrix3d cam_2_R,
                 double focal_length, double aspect_ratio, double scale) {

  scale = 0.2 * ((cam_1_t - cam_2_t).norm());

  // double dx = scale / 3.0;
  // double dy = scale / 3.0;
  double w = scale * 0.5;
  double z = scale * 0.5;
  double h = scale * 0.5;
  glMatrixMode(GL_MODELVIEW);
  Eigen::Matrix4d R_t;
  mat_R_t_from_R_and_t(cam_1_t, cam_1_R, R_t);

  float transform[16];
  for (size_t i = 0; i < 16; i++) {
    transform[i] = R_t(i / 4, i % 4);
  }

  glMultMatrixf(transform);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();
  // glVertex3f(origin(0), origin(1), origin(2));
  // glVertex3f(origin(0) + direction(0) + dx,
  //            origin(1) + scale * direction(1) + dy,
  //            origin(2) + scale * direction(2));

  // glVertex3f(origin(0), origin(1), origin(2));
  // glVertex3f(origin(0) + scale * direction(0) - dx,
  //            origin(1) + direction(1) + dy, origin(2) + scale *
  //            direction(2));

  // glVertex3f(origin(0), origin(1), origin(2));
  // glVertex3f(origin(0) + scale * direction(0) - dx,
  //            origin(1) + direction(1) - dy, origin(2) + scale *
  //            direction(2));

  // glVertex3f(origin(0), origin(1), origin(2));
  // glVertex3f(origin(0) + scale * direction(0) + dx,
  //            origin(1) + direction(1) - dy, origin(2) + scale *
  //            direction(2));

  // glVertex3f(origin(0) + scale * direction(0) + dx,
  //            origin(1) + direction(1) + dy, origin(2) + scale *
  //            direction(2));
  // glVertex3f(origin(0) + scale * direction(0) - dx,
  //            origin(1) + direction(1) + dy, origin(2) + scale *
  //            direction(2));

  // glVertex3f(origin(0) + scale * direction(0) + dx,
  //            origin(1) + direction(1) + dy, origin(2) + scale *
  //            direction(2));
  // glVertex3f(origin(0) + scale * direction(0) + dx,
  //            origin(1) + direction(1) - dy, origin(2) + scale *
  //            direction(2));

  // glVertex3f(origin(0) + scale * direction(0) + dx,
  //            origin(1) + direction(1) - dy, origin(2) + scale *
  //            direction(2));
  // glVertex3f(origin(0) + scale * direction(0) - dx,
  //            origin(1) + direction(1) - dy, origin(2) + scale *
  //            direction(2));

  // glVertex3f(origin(0) + scale * direction(0) - dx,
  //            origin(1) + direction(1) + dy, origin(2) + scale *
  //            direction(2));
  // glVertex3f(origin(0) + scale * direction(0) - dx,
  //            origin(1) + direction(1) - dy, origin(2) + scale *
  //            direction(2));

  // glEnd();
}

void draw_cameras_from_trajectory(std::vector<Eigen::Vector3d> traj_points,
                                  std::vector<Eigen::Matrix3d> traj_rotations,
                                  double f, double a, double s) {
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    draw_camera(traj_points[i], traj_points[i + 1], traj_rotations[i],
                traj_rotations[i + 1], f, a, s);
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
