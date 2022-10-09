#ifndef UTILS_H_
#define UTILS_H_

#include <fstream>

#include <Eigen/Eigen>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

void mat_R_t_from_R_and_t(Eigen::Vector3d t, Eigen::Matrix3d R,
                          Eigen::Matrix4d &R_t);

void draw_camera(Eigen::Vector3d cam_1_t, Eigen::Vector3d cam_2_t,
                 Eigen::Matrix3d cam_1_R, Eigen::Matrix3d cam_2_R,
                 double focal_length, double aspect_ratio, double scale);

void draw_cameras_from_trajectory(std::vector<Eigen::Vector3d> traj_points,
                                  std::vector<Eigen::Matrix3d> traj_rotations,
                                  double f, double a, double s);

template <typename T> T abs(T x);

void draw_kps(cv::Mat &image, std::vector<Eigen::Vector2d> p0,
              std::vector<Eigen::Vector2d> p1);

void draw_kps(cv::Mat &image, std::vector<cv::Point2f> p0,
              std::vector<cv::Point2f> p1);

Eigen::Vector3d parse_translation(std::ifstream &in);

#endif // UTILS_H_
