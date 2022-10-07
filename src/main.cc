#include <Eigen/Eigen>
#include <glm/glm.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <stdio.h>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <cstring>

#include "map.hh"
#include "triangulate.hh"
#include "utils.hh"

typedef Eigen::Vector3d WorldPoint;
typedef Eigen::Vector2d ImagePoint;

int main(int argc, char **argv) {
  if (argc != 3) {
    printf("usage: main <video-file>.mp4\n");
    return -1;
  }
  cv::VideoCapture vidcap;
  vidcap.open(argv[1]);
  size_t max_count = std::stoi(argv[2]);

  std::vector<glm::vec3> points;

  cv::Point2f pp;
  pp.x = 620.0;
  pp.y = 188.0;

  std::vector<Eigen::Vector3d> world_point_clouds;
  WorldMap map(718.0, pp, 1500, world_point_clouds);
  cv::Mat image, image_c;

  bool has_new_frames = true;
  has_new_frames = vidcap.read(image_c);
  cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);
  size_t count = 0;
  while (has_new_frames && (count < max_count)) {
    cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);
    map.register_new_image(image);
    count++;
    has_new_frames = vidcap.read(image_c);
    has_new_frames = vidcap.read(image_c);
  }

  bool show_ba_problem = false;

  cv::Mat image_g;
  if (show_ba_problem) {

    for (auto &ba_tuple : map.ba_problem) {
      Eigen::Vector3d _world_point = map.world_points_ba[std::get<0>(ba_tuple)];
      auto _x = std::get<1>(ba_tuple);
      auto _y = std::get<2>(ba_tuple);
      Eigen::Vector<double, 6> _cam_pos = map.camera_rt[std::get<3>(ba_tuple)];

      Eigen::Vector2d _estimated_projection;
      reproject(_cam_pos.data(), _world_point.data(),
                _estimated_projection.data(), 718.0);
      std::cout << "Expected: " << _x << ", " << _y
                << ". Got: " << _estimated_projection[0] << ", "
                << _estimated_projection[1] << std::endl;
    }
  }
  if (false) {
    vidcap.open(argv[1]);

    count = 0;
    while (has_new_frames && count < max_count) {
      has_new_frames = vidcap.read(image_c);
      has_new_frames = vidcap.read(image_c);

      for (auto &ba_tuple : map.ba_problem) {
        if (std::get<3>(ba_tuple) == count) {
          auto p = cv::Point2f(std::get<1>(ba_tuple), std::get<2>(ba_tuple));
          cv::circle(image_c, p, 1, cv::Scalar(0, 255, 0), cv::FILLED,
                     cv::LINE_8);
          // cv::putText(image_c, std::to_string(std::get<0>(ba_tuple)), p,
          //             cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));
          std::cout << "Expected: " << std::get<1>(ba_tuple) << ", "
                    << std::get<2>(ba_tuple) << std::endl;
          auto snavely_repro_error_functor = SnavelyReprojectionError(
              std::get<2>(ba_tuple), std::get<1>(ba_tuple), 718.0);
          Eigen::Vector2d reprojection_residual;
          snavely_repro_error_functor(
              map.camera_rt[std::get<3>(ba_tuple)].data(),
              map.world_points_ba[std::get<0>(ba_tuple)].data(),
              reprojection_residual.data());
          std::cout << "Residual: " << reprojection_residual << std::endl;
        }
      }

      cv::cvtColor(image_c, image_g, cv::COLOR_BGR2GRAY);
      cv::cvtColor(image_c, image_g, cv::COLOR_BGR2GRAY);
      cv::imshow("ba_diagnostics", image_c);
      char c = cv::waitKey(0);
      count++;
      if (c == 'q') {
        break;
      }
    }
  }

  cv::destroyAllWindows();

  std::cout << "bap Size: " << map.ba_problem.size() << "\n";

  float res = create_and_solve_ba_problem(map.ba_problem, map.world_points_ba,
                                          map.camera_rt);

  std::cout << "Visualizing " << map.world_points_ba.size() << " points."
            << std::endl;
  std::cout << "Trajectory Size: " << map.traj_points.size() << std::endl;
  // std::cout << "BA ERROR::" << res << std::endl;
  pangolin::CreateWindowAndBind("Renderer", 640, 480);
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0.5, -3, 0, 0, 0, pangolin::AxisY));
  pangolin::Renderable tree;
  for (size_t i = 0; i < 10; ++i) {
    auto axis_i = std::make_shared<pangolin::Axis>();
    axis_i->T_pc = pangolin::OpenGlMatrix::Translate(i * 20, i * 0.1, 0.0);
    tree.Add(axis_i);
  }
  pangolin::SceneHandler handler(tree, s_cam);
  pangolin::View &d_cam = pangolin::CreateDisplay().SetHandler(&handler);

  int idx = 0;

  std::cout << map.world_points_ba[0] << std::endl;

  while (!pangolin::ShouldQuit()) {
    idx++;
    // Clear the screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(1.0, 1.0, 1.0);

    glPointSize(1);
    pangolin::glDrawPoints(map.world_points_ba);
    // pangolin::glDrawPoints(std::vector<Eigen::Vector3d>(
    //     map.world_points_ba.begin(),
    //     map.world_points_ba.begin() + (int)(idx * 1000)));
    glColor3f(0.0, 0.0, 1.0);
    glPointSize(3);
    pangolin::glDrawPoints(map.traj_points);
    pangolin::glDrawVertices(map.traj_points, GL_LINE_STRIP);
    glPointSize(1);
    Eigen::Vector3d one, two;
    glColor3f(1.0, 0.0, 0.0);
    // draw_cameras_from_trajectory(map.traj_points, map.traj_rotations,
    // map.focal, map.pp.y / map.pp.x, 10.0);
    d_cam.Activate(s_cam);
    pangolin::FinishFrame();

    // if (idx % 1000 == 0) {

    //  float res = create_and_solve_ba_problem(
    //      map.ba_problem, map.world_points_ba, map.camera_rt);
    //  std::cout << map.traj_points[7] << std::endl;
    //}
  }

  return 0;
}
