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

#include <ceres/ceres.h>
#include <ceres/rotation.h>

typedef Eigen::Vector3d WorldPoint;
typedef Eigen::Vector2d ImagePoint;

class Frame {
public:
  cv::Mat image;
  // all the matched projections in this image
  std::vector<ImagePoint> projected_points;
  // map from index in Frame::projected_points to WorldPoint id
  std::unordered_map<size_t, size_t> correspondences;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
};

uint32_t hash_keypoint(double u, double v) {
  uint16_t iu = (int)u;
  uint16_t iv = (int)v;

  return (((uint32_t)iu) << 16) + (uint32_t)iv;
}

uint32_t hash_keypoint(cv::KeyPoint kp) {
  return hash_keypoint(kp.pt.x, kp.pt.y);
}

uint32_t hash_point2f(cv::Point2f p2f) { return hash_keypoint(p2f.x, p2f.y); }

void print_map(std::map<uint32_t, size_t> &myMap) {
  for (auto it = myMap.cbegin(); it != myMap.cend(); ++it) {
    std::cout << it->first << " " << it->second << "\n";
  }
}
class WorldMap {
public:
  std::vector<Frame> frames;
  cv::Mat img_old, img_new;
  std::vector<cv::Point2f> points_old, points_new;
  double focal;
  cv::Point2f pp;
  size_t min_points_per_frame;
  std::vector<WorldPoint> world_points_clouds;
  Eigen::Matrix3d Ra, R;
  Eigen::Vector3d ta, t;
  std::vector<WorldPoint> traj_points;
  std::vector<Eigen::Matrix3d> traj_rotations;
  std::vector<Eigen::Vector3d> traj_rots_a;
  cv::Mat draw_img;
  std::vector<cv::KeyPoint> keypoints_new, keypoints_old;
  cv::Mat descriptors_new, descriptors_old;
  std::vector<ImagePoint> imagepoints_old, imagepoints_new;
  std::vector<WorldPoint> world_points_in_this_iteration;

  std::vector<Eigen::Vector2d> image_points;
  std::vector<Eigen::Vector3d> world_points_ba;
  std::map<uint32_t, size_t> keypoint_pt_to_world_point_index;

  // world points, image points, pose indices
  std::vector<std::tuple<size_t, double, double, size_t>> ba_problem;

  std::vector<Eigen::Vector<double, 6>> camera_rt;

public:
  WorldMap(double focal, cv::Point2f pp, size_t min_points,
           std::vector<Eigen::Vector3d> &world_point_clouds_in) {
    Ra = Eigen::Matrix3d::Identity();
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();
    ta = Eigen::Vector3d::Zero();
    this->focal = focal;
    this->pp = pp;
    this->min_points_per_frame = min_points;
    this->world_points_clouds = world_point_clouds_in;
  }

  bool register_new_image(cv::Mat &new_img) {
    auto feature_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
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

    std::cout << "Frame: " << this->frames.size() << std::endl;

    keypoints_new.clear();
    this->descriptors_new.release();

    detector->detectAndCompute(this->img_new, cv::Mat(), keypoints_new,
                               descriptors_new);

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
    cv::imshow("kps_1", draw_img);
    cv::waitKey(1);

    cv::Mat world_points_mat;
    cv::Mat R_mat, t_mat;

    std::cout << "Tried to triangulate" << points_new.size() << " points"
              << std::endl;
    size_t s = triangulate_points(points_old, points_new, focal, pp, R_mat,
                                  t_mat, world_points_mat);
    if (s == 0) {
      return true;
    }

    std::cout << "Triangulated" << world_points_mat.size() << " points"
              << std::endl;

    this->world_points_in_this_iteration.clear();

    Eigen::Vector3d world_point;
    Eigen::Vector3d camera_axis;
    camera_axis << 0, 0, -1;
    for (int i = 0; i < world_points_mat.rows; i++) {
      world_point = Ra * Eigen::Vector3d(world_points_mat.at<float>(i, 0),
                                         world_points_mat.at<float>(i, 1),
                                         world_points_mat.at<float>(i, 2)) +
                    ta;
      if ((Ra * camera_axis).dot(ta - world_point) < 0)
        continue;
      if ((ta - world_point).norm() > 300)
        continue;
      this->world_points_clouds.push_back(world_point);
      this->world_points_in_this_iteration.push_back(world_point);
    }

    cv::cvtColor(img_new, draw_img, cv::COLOR_GRAY2BGR);
    draw_kps(draw_img, points_old, points_new);
    cv::imshow("kps_2", draw_img);
    cv::waitKey(1);

    R << R_mat.at<double>(0, 0), R_mat.at<double>(0, 1), R_mat.at<double>(0, 2),
        R_mat.at<double>(1, 0), R_mat.at<double>(1, 1), R_mat.at<double>(1, 2),
        R_mat.at<double>(2, 0), R_mat.at<double>(2, 1), R_mat.at<double>(2, 2);
    t << t_mat.at<double>(0), t_mat.at<double>(1), t_mat.at<double>(2);
    ta = ta + Ra * t;
    Ra = R * (Ra);

    Eigen::Vector3d Raa;
    ceres::RotationMatrixToAngleAxis(Ra.data(), Raa.data());

    traj_points.push_back(ta);
    traj_rotations.push_back(Ra);
    traj_rots_a.push_back(Raa);

    Eigen::Vector<double, 6> camera_pose;
    camera_pose << Raa[0], Raa[1], Raa[2], ta[0], ta[1], ta[2];
    this->camera_rt.push_back(camera_pose);

    std::map<uint32_t, size_t> old_associative_index;

    int found = 0, notfound = 0;

    for (size_t i = 0; i < world_points_in_this_iteration.size(); i++) {
      if (this->keypoint_pt_to_world_point_index.count(
              hash_point2f(this->points_old[i]))) {
        found++;
        this->ba_problem.push_back(
            {this->keypoint_pt_to_world_point_index[hash_point2f(
                 this->points_old[i])], // takes i to a world
                                        // point
             this->points_new[i].x,     // where the projection was found
             this->points_new[i].y,     // same but y
             this->traj_rots_a.size() -
                 1}); // index to traj_rots_a and traj_points
        old_associative_index[hash_keypoint(this->keypoints_new[i])] =
            this->keypoint_pt_to_world_point_index[hash_keypoint(
                this->keypoints_old[i])];
      } else {
        notfound++;
        this->world_points_ba.push_back(
            this->world_points_in_this_iteration[i]);
        old_associative_index[hash_keypoint(this->keypoints_new[i])] =
            world_points_ba.size() - 1;
      }
    }

    std::cout << "Found: " << found << ", notfound: " << notfound
              << ", mapsize: " << old_associative_index.size() << std::endl;

    this->keypoint_pt_to_world_point_index = old_associative_index;

    this->keypoints_old = this->keypoints_new;
    this->descriptors_old = this->descriptors_new;
    this->img_old = this->img_new;
    return true;
  }
};

struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y,
                           double focal_len)
      : observed_x(observed_x), observed_y(observed_y), focal_len(focal_len) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const point,
                  T *residuals) const {
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    T predicted_x = focal_len * xp;
    T predicted_y = focal_len * yp;

    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }

  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y,
                                     const double focal_len) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
        new SnavelyReprojectionError(observed_x, observed_y, focal_len)));
  }
  double observed_x, observed_y, focal_len;
};

float create_and_solve_ba_problem(
    std::vector<std::tuple<size_t, double, double, size_t>> &bap,
    std::vector<WorldPoint> &world_points,
    std::vector<Eigen::Vector<double, 6>> &traj_poses) {

  double focal = 718;
  ceres::Problem problem;

  for (size_t i = 0; i < bap.size(); ++i) {
    ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(
        std::get<1>(bap[i]), std::get<2>(bap[i]), focal
        // add points from bap
    );

    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                             (traj_poses[std::get<3>(bap[i])].data()),
                             (world_points[std::get<0>(bap[i])]).data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}

#endif // MAP_H_
