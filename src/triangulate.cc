#include "../inc/triangulate.hh"

int filter_using_mask(std::vector<cv::Point2f> &in_vector,
                      std::vector<uchar> mask) {
  size_t i, k;
  for (i = k = 0; i < mask.size(); i++) {
    if (mask[i] == 0)
      continue;
    in_vector[k++] = in_vector[i];
  }
  in_vector.resize(k);
  return k;
}

int triangulate_points(std::vector<cv::Point2f> &new_points,
                       std::vector<cv::Point2f> &old_points, double focal,
                       cv::Point2d pp, cv::Mat &R, cv::Mat &t,
                       cv::Mat &world_points) {

  std::vector<uchar> mask;
  cv::Mat essential_matrix = cv::findEssentialMat(
      new_points, old_points, focal, pp, cv::RANSAC, 0.999, 1.0, 1000, mask);
  size_t s;
  // std::cout << "Essential Matrix: " << essential_matrix << std::endl;
  filter_using_mask(new_points, mask);
  s = filter_using_mask(old_points, mask);

  if (s == 0)
    return s;

  // std::cout << "Size of points after filter: " << s << std::endl;
  // recoverPose
  mask.clear();
  cv::recoverPose(essential_matrix, new_points, old_points, R, t, focal, pp,
                  mask);
  // std::cout << "Pose: " << R << "    " << t << std::endl;
  filter_using_mask(new_points, mask);
  // std::cout << mask << std::endl;
  s = filter_using_mask(old_points, mask);

  if (s == 0)
    return s;

  // std::cout << "Size of points after filter: " << s << std::endl;
  // triangulatePoints
  cv::Mat proj_mat_1, proj_mat_2, points4D;
  // std::cout << CV_32F << " CV_32F\n";
  cv::Mat R_t(cv::Size(4, 3), CV_32F, 0.0);
  cv::Mat R_t_2(cv::Size(4, 3), CV_32F, 0.0);
  cv::Mat intrinsics(cv::Size(3, 3), CV_32F, 0.0);

  R_t.at<float>(0, 0) = 1;
  R_t.at<float>(1, 1) = 1;
  R_t.at<float>(2, 2) = 1;

  R.copyTo(R_t_2(cv::Range::all(), cv::Range(0, 3)));
  t.copyTo(R_t_2.col(3));

  intrinsics.at<float>(0, 0) = focal;
  intrinsics.at<float>(1, 1) = focal;
  intrinsics.at<float>(0, 2) = pp.x;
  intrinsics.at<float>(1, 2) = pp.y;
  intrinsics.at<float>(2, 2) = 1.0;

  cv::Mat projectionMatrix_1 = intrinsics * R_t;
  cv::Mat projectionMatrix_2 = intrinsics * R_t_2;

  cv::Mat world_points_m;

  cv::triangulatePoints(projectionMatrix_1, projectionMatrix_2, old_points,
                        new_points, world_points_m);

  cv::convertPointsFromHomogeneous(world_points_m.t(), world_points);
  return s;
}
