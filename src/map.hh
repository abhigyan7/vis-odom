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
  cv::Mat img_1, img_2;
  std::vector<cv::Point2f> points_1;
  std::vector<cv::Point2f> points_2;
  double focal;
  cv::Point2f pp;
  size_t min_points_per_frame;
  std::vector<Eigen::Vector3f> world_points_clouds;
  Eigen::Matrix3f Ra, R;
  Eigen::Vector3f ta, t;
  std::vector<Eigen::Vector3f> traj_points;
  std::vector<Eigen::Matrix3f> traj_rotations;
  cv::Mat draw_img;
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  cv::Mat descriptors_1, descriptors_2;
  std::vector<ImagePoint> imagepoints_1, imagepoints_2;

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
      this->img_1 = new_img.clone();

      detector->detect(this->img_1, keypoints_1, cv::Mat());
      detector->detectAndCompute(this->img_1, cv::Mat(), keypoints_1,
                                 descriptors_1);
      std::cout << "Descriptors" << descriptors_1 << std::endl;
      std::cout << "Shape of keypoints: " << keypoints_1.size() << std::endl;
      std::cout << "Shape: " << descriptors_1.size() << std::endl;
      return true;
    }

    Frame frame_2;
    frame_2.image = new_img;
    this->frames.push_back(frame_2);
    this->img_2 = new_img.clone();

    std::vector<uchar> status;
    std::vector<float> error;

    detector->detect(this->img_1, keypoints_1, cv::Mat());
    detector->compute(this->img_1, keypoints_1, descriptors_1);

    // detector->detect(this->img_2, keypoints_2, cv::Mat());
    detector->detectAndCompute(this->img_2, cv::Mat(), keypoints_2,
                               descriptors_2);

    std::vector<cv::DMatch> matches;
    feature_matcher->match(descriptors_1, descriptors_2, matches);

    std::vector<cv::Point2f> points_1, points_2;
    std::vector<cv::Point2f> temp_points_1, temp_points_2;
    cv::KeyPoint::convert(keypoints_1, temp_points_1);
    cv::KeyPoint::convert(keypoints_2, temp_points_2);

    for (auto match : matches) {
      points_1.push_back(temp_points_1[match.queryIdx]);
      points_2.push_back(temp_points_2[match.trainIdx]);
    }

    std::cout << "Pushed frame: " << this->frames.size() << ", ";
    cv::cvtColor(img_1, draw_img, cv::COLOR_GRAY2BGR);
    // draw_kps(draw_img, points_1, points_2);
    //  cv::imshow("kps_1", draw_img);
    cv::cvtColor(img_2, draw_img, cv::COLOR_GRAY2BGR);
    draw_kps(draw_img, points_1, points_2);
    cv::imshow("kps_2", draw_img);
    cv::waitKey(1);

    size_t i, k;
    for (i = k = 0; i < this->points_1.size(); i++) {
      // if (!status[i])
      // continue;
      this->points_1[k++] = this->points_1[i];
      this->points_2[k] = this->points_2[i];
    }

    this->points_1.resize(k);
    this->points_2.resize(k);

    this->img_1 = img_2;

    cv::Mat world_points_mat;
    cv::Mat R_mat, t_mat;
    triangulate_points(points_1, points_2, focal, pp, R_mat, t_mat,
                       world_points_mat);

    Eigen::Vector3f world_point;
    Eigen::Vector3f camera_axis;
    camera_axis << 0, 0, -1;
    for (int i = 0; i < world_points_mat.rows; i++) {
      world_point = Ra * Eigen::Vector3f(world_points_mat.at<float>(i, 0),
                                         world_points_mat.at<float>(i, 1),
                                         world_points_mat.at<float>(i, 2)) +
                    ta;
      if ((Ra * camera_axis).dot(ta - world_point) < 0)
        continue;
      if ((ta - world_point).norm() > 30)
        continue;
      this->world_points_clouds.push_back(world_point);
    }
    R << R_mat.at<double>(0, 0), R_mat.at<double>(0, 1), R_mat.at<double>(0, 2),
        R_mat.at<double>(1, 0), R_mat.at<double>(1, 1), R_mat.at<double>(1, 2),
        R_mat.at<double>(2, 0), R_mat.at<double>(2, 1), R_mat.at<double>(2, 2);
    t << t_mat.at<double>(0), t_mat.at<double>(1), t_mat.at<double>(2);
    ta = ta + Ra * t;
    Ra = R * (Ra);

    std::cout << "Trajectory: " << std::endl << ta << std::endl;
    std::cout << "Rotation: " << std::endl << R_mat << std::endl;

    traj_points.push_back(ta);
    traj_rotations.push_back(Ra);

    return true;
  }
};

#endif // MAP_H_
