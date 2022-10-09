#ifndef MAP_H_
#define MAP_H_

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <stdint.h>
#include <stdio.h>

#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glm/glm.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "triangulate.hh"
#include "utils.hh"

typedef Eigen::Vector3d WorldPoint;
typedef Eigen::Vector2d ImagePoint;
typedef Eigen::Matrix3d RotationMatrix;

class Frame {
public:
  size_t id;
  Eigen::Matrix3d R;
  WorldPoint t;

  Frame(size_t _id, Eigen::Matrix3d _R, WorldPoint _t)
      : id(_id), R(_R), t(_t) {}
};

class Correspondence {
public:
  size_t world_point_idx;
  size_t frame_idx;
  ImagePoint image_point;
};

class WorldMap {
public:
  std::vector<Correspondence> correspondences;
  std::vector<Frame> frames;
  std::vector<WorldPoint> world_points;

  size_t min_points_per_frame;
  std::vector<cv::Point2f> points_old, points_new;
  std::vector<cv::KeyPoint> keypoints_new, keypoints_old;
  cv::Mat descriptors_new, descriptors_old;
  cv::Mat img_old, img_new, draw_img;
  Eigen::Matrix3d Ra, R;
  Eigen::Vector3d ta, t;

  std::vector<WorldPoint> traj_points;

  cv::Point2f pp;
  double focal_length;

public:
  WorldMap(float focal_length, cv::Point2f pp, size_t min_points)
      : focal_length(focal_length), pp(pp), min_points_per_frame(min_points) {

    this->Ra = Eigen::Matrix3d::Identity();
    this->ta = Eigen::Vector3d::Zero();
    this->R = Eigen::Matrix3d::Identity();
    this->t = Eigen::Vector3d::Zero();
  }

  bool register_new_image(cv::Mat &new_img, float translation_norm = 1.0) {
    auto feature_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    int fast_threshold = 20;
    bool nonmaxSupression = true;
    auto detector = cv::ORB::create(this->min_points_per_frame, 1.2f, 8, 31, 0,
                                    2, cv::ORB::FAST_SCORE);

    if (this->frames.size() == 0) {
      this->img_old = new_img.clone();

      cv::FAST(new_img, keypoints_old, fast_threshold, nonmaxSupression);
      detector->compute(this->img_old, keypoints_old, descriptors_old);
      this->frames.push_back(Frame(this->frames.size(), this->Ra, this->ta));
      return true;
    }

    this->img_new = new_img.clone();

    std::vector<uchar> status;
    std::vector<float> error;

    keypoints_new.clear();
    this->descriptors_new.release();

    cv::FAST(img_new, keypoints_new, fast_threshold, nonmaxSupression);
    detector->compute(this->img_new, keypoints_new, descriptors_new);

    std::vector<std::vector<cv::DMatch>> raw_matches;
    feature_matcher->knnMatch(descriptors_new, descriptors_old, raw_matches, 2);

    std::vector<cv::DMatch> matches;

    float nn_match_ratio = 0.8f;
    for (size_t i = 0; i < raw_matches.size(); i++) {
      cv::DMatch first = raw_matches[i][0];
      float dist1 = raw_matches[i][0].distance;
      float dist2 = raw_matches[i][1].distance;
      if (dist1 < nn_match_ratio * dist2) {
        matches.push_back(first);
      }
    }

    std::vector<cv::Point2f> temp_points_old, temp_points_new;
    cv::KeyPoint::convert(keypoints_old, temp_points_old);
    cv::KeyPoint::convert(keypoints_new, temp_points_new);

    points_old.clear();
    points_new.clear();
    for (auto match : matches) {
      points_old.push_back(temp_points_old[match.trainIdx]);
      points_new.push_back(temp_points_new[match.queryIdx]);
    }

    cv::cvtColor(img_new, draw_img, cv::COLOR_GRAY2BGR);
    draw_kps(draw_img, points_old, points_new);

    cv::Mat world_points_mat;
    cv::Mat R_mat, t_mat;

    size_t s = triangulate_points(points_new, points_old, focal_length, pp,
                                  R_mat, t_mat, world_points_mat);

    if (s == 0) {
      return true;
    }

    Eigen::Vector3d world_point;
    Eigen::Vector3d camera_axis;
    camera_axis << 0, 0, -1;
    std::vector<size_t> world_points_indices_for_this_frame;
    std::vector<WorldPoint> triangulated_points;
    for (int i = 0; i < world_points_mat.rows; i++) {
      world_point = Ra * Eigen::Vector3d(world_points_mat.at<float>(i, 0),
                                         world_points_mat.at<float>(i, 1),
                                         world_points_mat.at<float>(i, 2)) +
                    ta;
      if ((Ra * camera_axis).dot(ta - world_point) < 0)
        continue;
      if ((ta - world_point).norm() > 30)
        continue;
      world_points_indices_for_this_frame.push_back(this->world_points.size());
      triangulated_points.push_back(world_point);
      this->world_points.push_back(world_point);
    }

    cv::cvtColor(img_new, draw_img, cv::COLOR_GRAY2BGR);
    draw_kps(draw_img, points_old, points_new);
    cv::rectangle(draw_img, cv::Rect2i(0, 0, 300, 40),
                  cv::Scalar(240, 240, 240), -1);
    std::string info_string = "f_id: ";
    info_string += std::to_string(this->frames.size());
    info_string += ", ";
    info_string += std::to_string(s);
    info_string += " points";
    cv::putText(draw_img, info_string, cv::Point(0, 30),
                cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 0, 0), 2.0);
    cv::imshow("kps_2", draw_img);
    cv::waitKey(1);

    R << R_mat.at<double>(0, 0), R_mat.at<double>(0, 1), R_mat.at<double>(0, 2),
        R_mat.at<double>(1, 0), R_mat.at<double>(1, 1), R_mat.at<double>(1, 2),
        R_mat.at<double>(2, 0), R_mat.at<double>(2, 1), R_mat.at<double>(2, 2);
    t << t_mat.at<double>(0), t_mat.at<double>(1), t_mat.at<double>(2);

    if (translation_norm > 0.1) {
      t = t * (1 / t.norm()) * translation_norm;
      ta = ta + Ra * t;
      Ra = R * (Ra);
      std::cout << "Frame: " << this->frames.size();
      std::cout << ", Translation: (" << ta[0] << ", " << ta[1] << ", " << ta[2]
                << ")" << std::endl;
    }

    traj_points.push_back(ta);
    this->frames.push_back(Frame(this->frames.size(), this->Ra, this->ta));

    this->img_old = this->img_new;
    this->keypoints_old = this->keypoints_new;
    this->descriptors_old = this->descriptors_new;

    return true;
  }
};

struct PerspectiveProjectionError {
  PerspectiveProjectionError(double observed_x, double observed_y,
                             double focal_len, double pp_x, double pp_y)
      : observed_x(observed_x), observed_y(observed_y), focal_len(focal_len),
        pp_x(pp_x), pp_y(pp_y) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const point,
                  T *residuals) const {
    T p1[3], p2[3], rot[3];

    p1[0] = point[0] - camera[3];
    p1[1] = point[1] - camera[4];
    p1[2] = point[2] - camera[5];

    rot[0] = -camera[0];
    rot[1] = -camera[1];
    rot[2] = -camera[2];

    ceres::AngleAxisRotatePoint(rot, p1, p2);

    T xp = p2[0] / p2[2];
    T yp = p2[1] / p2[2];

    T predicted_x = focal_len * xp + pp_x;
    T predicted_y = focal_len * yp + pp_y;

    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y,
                                     const double focal_len, const double pp_x,
                                     const double pp_y) {
    return (
        new ceres::AutoDiffCostFunction<PerspectiveProjectionError, 2, 6, 3>(
            new PerspectiveProjectionError(observed_x, observed_y, focal_len,
                                           pp_x, pp_y)));
  }
  double observed_x, observed_y, focal_len, pp_x, pp_y;
};

float create_and_solve_ba_problem(
    std::vector<std::tuple<size_t, double, double, size_t>> &bap,
    std::vector<WorldPoint> &world_points,
    std::vector<Eigen::Vector<double, 6>> &traj_poses, double f, double pp_x,
    double pp_y);

#endif // MAP_H_
