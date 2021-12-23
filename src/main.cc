#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "triangulate.hh"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <stdio.h>

// DONE create constructors
// DONE triangulate points from image pairs
// TODO Create correspondences from optical flow results
// TODO Create associated pose graph-y thing using optical flow - mask off
// existing points when searching for new ones
// TODO build pose graph from triangulated data
// TODO encapsulate triangulation state into a class

typedef Eigen::Vector3f WorldPoint;
typedef Eigen::Vector2f ImagePoint;

class Frame {
public:
  cv::Mat image;
  // maps point id (WorldPoint.id) to its projection in this frame
  std::unordered_map<int, ImagePoint> projected_points;
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
};

class WorldMap {
public:
  std::vector<Frame> frames;
  cv::Mat img_1, img_2;
  std::vector<cv::Point2f> points_1;
  std::vector<cv::Point2f> points_2;
  double focal;
  cv::Point2f pp;
  size_t min_points_per_frame;
  std::vector<cv::Mat> world_points_clouds;

public:
  WorldMap(double focal, cv::Point2f pp, size_t min_points,
           std::vector<cv::Mat> &world_point_clouds_in) {
    this->focal = focal;
    this->pp = pp;
    this->min_points_per_frame = min_points;
    this->world_points_clouds = world_point_clouds_in;
  }

  bool register_new_image(cv::Mat &new_img) {

    if (this->frames.size() == 0) {
      Frame frame_1;
      frame_1.image = new_img;
      this->frames.push_back(frame_1);
      this->img_1 = new_img;

      return true;
    }

    Frame frame_2;
    frame_2.image = new_img;
    this->frames.push_back(frame_2);
    this->img_2 = new_img;
    this->img_2 = new_img;

    auto feature_matcher = cv::BFMatcher::create();
    auto detector = cv::ORB::create();

    std::vector<uchar> status;
    std::vector<float> error;

    std::vector<cv::KeyPoint> keypoints_1;
    std::vector<cv::KeyPoint> keypoints_2;
    cv::Mat descriptors_1;
    cv::Mat descriptors_2;
    detector->detect(this->img_1, keypoints_1, cv::Mat());
    detector->detect(this->img_2, keypoints_2, cv::Mat());
    detector->detectAndCompute(this->img_1, cv::Mat(), keypoints_1,
                               descriptors_1);
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
    this->img_1 = img_2;

    size_t i, k;
    for (i = k = 0; i < this->points_1.size(); i++) {
      if (!status[i])
        continue;
      this->points_1[k++] = this->points_1[i];
      this->points_2[k] = this->points_2[i];
    }

    this->points_1.resize(k);
    this->points_2.resize(k);

    std::cout << "number of points: " << points_1.size() << std::endl;

    cv::Mat R, t;

    cv::Mat world_points_mat;
    triangulate_points(points_1, points_2, focal, pp, R, t, world_points_mat);

    this->world_points_clouds.push_back(world_points_mat);

    // cv::vconcat(this->world_points, world_points_mat, this->world_points);

    return true;
  }
};

int main(int argc, char **argv) {
  if (argc != 2) {
    printf("usage: main <video-file>.mp4\n");
    return -1;
  }
  cv::VideoCapture vidcap;
  vidcap.open(argv[1]);
  polyscope::init();
  std::vector<glm::vec3> points;

  cv::Point2f pp;
  pp.x = 0.0;
  pp.y = 0.0;

  std::vector<cv::Mat> world_point_clouds;
  WorldMap map(700.0, pp, 2500, world_point_clouds);
  cv::Mat image, image_c;

  bool has_new_frames = true;
  has_new_frames = vidcap.read(image_c);
  cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);
  map.register_new_image(image);
  size_t count = 0;
  while (has_new_frames && count < 10) {
    cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);
    map.register_new_image(image);
    has_new_frames = vidcap.read(image_c);
    // count++;
  }

  cv::Mat world_points;
  cv::Mat R, t;
  // triangulate_points(image_1, image_2, points_1, 700.0, pp, R, t,
  // world_points);
  //  visualize!
  //
  //

  std::cout << "No of PCs: " << map.world_points_clouds.size() << std::endl;
  int iii = 0;
  for (auto point_cloud : map.world_points_clouds) {
    std::vector<glm::vec3> world_glm;
    iii++;
    for (int i = 0; i < point_cloud.rows; i++) {
      world_glm.push_back(glm::vec3(point_cloud.at<float>(i, 0),
                                    point_cloud.at<float>(i, 1),
                                    point_cloud.at<float>(i, 2)));
    }
    polyscope::PointCloud *psCloud =
        polyscope::registerPointCloud(std::to_string(iii), world_glm);
    psCloud->setPointRadius(0.0002);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
  }
  // for (int i = 0; i < world_points.rows; i++) // {
  //   world_glm.push_back(glm::vec3(world_points.at<float>(i, 0),
  //                                 world_points.at<float>(i, 1),
  //                                 world_points.at<float>(i, 2)));
  // }

  // set some options
  polyscope::show();
  return 0;
}
