#ifndef MAP_H_
#define MAP_H_

#include "utils.hh"
#include <Eigen/Eigen>
#include <glm/glm.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <stdio.h>

#include "triangulate.hh"

#include <ceres/rotation.h>

typedef Eigen::Vector3f WorldPoint;
typedef Eigen::Vector2f ImagePoint;

class Frame {
public:
  cv::Mat image;
  // all the matched projections in this image
  std::vector<ImagePoint> projected_points;
  // map from index in Frame::projected_points to WorldPoint id
  std::unordered_map<size_t, size_t> correspondences;
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
};

class WorldMap {
public:
  std::vector<Frame> frames;
  std::vector<WorldPoint> world_points;
  cv::Mat img_old, img_new;
  std::vector<cv::Point2f> points_old, points_new;
  double focal;
  cv::Point2f pp;
  size_t min_points_per_frame;
  std::vector<Eigen::Vector3f> world_points_clouds;
  Eigen::Matrix3f Ra, R;
  Eigen::Vector3f ta, t;
  std::vector<Eigen::Vector3f> traj_points;
  std::vector<Eigen::Matrix3f> traj_rotations;
  std::vector<Eigen::Vector3f> traj_rots_a;
  cv::Mat draw_img;
  std::vector<cv::KeyPoint> keypoints_new, keypoints_old;
  cv::Mat descriptors_new, descriptors_old;
  std::vector<ImagePoint> imagepoints_old, imagepoints_new;

public:
  WorldMap(double focal, cv::Point2f pp, size_t min_points,
           std::vector<Eigen::Vector3f> &world_point_clouds_in) {
    Ra = Eigen::Matrix3f::Identity();
    R = Eigen::Matrix3f::Identity();
    t = Eigen::Vector3f::Zero();
    ta = Eigen::Vector3f::Zero();
    this->focal = focal;
    this->pp = pp;
    this->min_points_per_frame = min_points;
    this->world_points_clouds = world_point_clouds_in;
  }

  bool register_new_image(cv::Mat &new_img) {
    auto feature_matcher = cv::BFMatcher::create();
    auto detector = cv::ORB::create(2000);
    if (this->frames.size() == 0) {
      Frame frame_1;
      frame_1.image = new_img;
      this->frames.push_back(frame_1);
      this->img_old = new_img.clone();

      detector->detectAndCompute(this->img_old, cv::Mat(), keypoints_old,
                                 descriptors_old);
      return true;
    }

    Frame frame_2;
    frame_2.image = new_img;
    this->frames.push_back(frame_2);
    this->img_new = new_img.clone();

    std::vector<uchar> status;
    std::vector<float> error;

    keypoints_new.clear();
    keypoints_old.clear();
    this->descriptors_old.release();
    this->descriptors_new.release();

    // detector->detect(this->img_2, keypoints_2, cv::Mat());
    detector->detectAndCompute(this->img_old, cv::Mat(), keypoints_old,
                               descriptors_old);
    detector->detectAndCompute(this->img_new, cv::Mat(), keypoints_new,
                               descriptors_new);

    std::vector<cv::DMatch> matches;
    feature_matcher->match(descriptors_new, descriptors_old, matches);

    std::cout << "Matched " << matches.size() << " points" << std::endl;

    std::vector<cv::Point2f> temp_points_old, temp_points_new;
    cv::KeyPoint::convert(keypoints_old, temp_points_old);
    cv::KeyPoint::convert(keypoints_new, temp_points_new);

    points_old.clear();
    points_new.clear();
    for (auto match : matches) {
      points_old.push_back(temp_points_old[match.trainIdx]);
      points_new.push_back(temp_points_new[match.queryIdx]);
    }

    std::cout << "Pushed frame: " << this->frames.size() << ", ";
    cv::cvtColor(img_new, draw_img, cv::COLOR_GRAY2BGR);
    draw_kps(draw_img, points_old, points_new);
    cv::imshow("kps_2", draw_img);
    cv::waitKey(10);

    // size_t i, k;
    // for (i = k = 0; i < this->points_old.size(); i++) {
    //   // if (!status[i])
    //   // continue;
    //   this->points_old[k++] = this->points_old[i];
    //   this->points_new[k] = this->points_new[i];
    // }

    // this->points_old.resize(k);
    // this->points_new.resize(k);

    cv::Mat world_points_mat;
    cv::Mat R_mat, t_mat;

    std::cout << "Tried to triangulate" << points_new.size() << " points"
              << std::endl;
    triangulate_points(points_new, points_old, focal, pp, R_mat, t_mat,
                       world_points_mat);

    std::cout << "Triangulated" << world_points_mat.size() << " points"
              << std::endl;
    Eigen::Vector3f world_point;
    Eigen::Vector3f camera_axis;
    camera_axis << 0, 0, -1;
    for (int i = 0; i < world_points_mat.rows; i++) {
      world_point = Ra * Eigen::Vector3f(world_points_mat.at<float>(i, 0),
                                         world_points_mat.at<float>(i, 1),
                                         world_points_mat.at<float>(i, 2)) +
                    ta;
      // if ((Ra * camera_axis).dot(ta - world_point) < 0)
      //   continue;
      // if ((ta - world_point).norm() > 30)
      //   continue;
      this->world_points_clouds.push_back(world_point);
    }
    R << R_mat.at<double>(0, 0), R_mat.at<double>(0, 1), R_mat.at<double>(0, 2),
        R_mat.at<double>(1, 0), R_mat.at<double>(1, 1), R_mat.at<double>(1, 2),
        R_mat.at<double>(2, 0), R_mat.at<double>(2, 1), R_mat.at<double>(2, 2);
    t << t_mat.at<double>(0), t_mat.at<double>(1), t_mat.at<double>(2);
    ta = ta + Ra * t;
    Ra = R * (Ra);

    Eigen::Vector3f Raa;
    ceres::RotationMatrixToAngleAxis(Ra.data(), Raa.data());
    std::cout << "ROTATION IN ANGLE AXIS:: ";
    std::cout << Raa << std::endl;

    traj_points.push_back(ta);
    traj_rotations.push_back(Ra);
    traj_rots_a.push_back(Raa);

    this->keypoints_old = this->keypoints_new;
    this->descriptors_old = this->descriptors_new;
    this->img_old = this->img_new;

    return true;
  }
};

#endif // MAP_H_
