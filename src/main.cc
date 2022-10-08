#include <Eigen/Eigen>
#include <fstream>
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

#include "../inc/map.hh"
#include "../inc/triangulate.hh"
#include "../inc/utils.hh"

typedef Eigen::Vector3d WorldPoint;
typedef Eigen::Vector2d ImagePoint;

int main(int argc, char **argv) {
  if (argc < 6) {
    printf("usage: main <video-file>.mp4 frame_id f cx cy n_features "
           "poses_file\n");
    return -1;
  }
  cv::VideoCapture vidcap;
  vidcap.open(argv[1]);
  size_t max_count = std::stoi(argv[2]);

  std::vector<glm::vec3> points;

  cv::Point2f pp;
  float focal_length;

  pp.x = std::stof(argv[4]);
  pp.y = std::stof(argv[5]);

  focal_length = std::stof(argv[3]);

  int n_features = std::stoi(argv[6]);

  std::cout << pp.x << ", " << pp.y << ", " << focal_length;

  std::string poses_file_name;
  bool poses_file_exists = false;

  float r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3;
  float last_t1 = 0, last_t2 = 0, last_t3 = 0;

  std::ifstream poses_file;
  if (argc > 7) {
    poses_file_name = argv[7];
    poses_file = std::ifstream(poses_file_name);
    poses_file_exists = true;
  }

  std::vector<Eigen::Vector3d> world_point_clouds;
  WorldMap map(focal_length, pp, n_features, world_point_clouds);
  cv::Mat image, image_c;

  bool has_new_frames = true;
  has_new_frames = vidcap.read(image_c);
  cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);
  size_t count = 0;
  while (has_new_frames && (count < max_count)) {
    cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);

    float translation_norm = 1.0;
    if (poses_file_exists) {
      poses_file >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >>
          r32 >> r33 >> t3;
      float _t1 = last_t1 - t1;
      float _t2 = last_t2 - t2;
      float _t3 = last_t3 - t3;
      translation_norm = sqrt(_t1 * _t1 + _t2 * _t2 + _t3 * _t3);
      last_t1 = t1;
      last_t2 = t2;
      last_t3 = t3;
      std::cout << "Translation scale: " << translation_norm << std::endl;
    }
    map.register_new_image(image, translation_norm);

    count++;
    has_new_frames = vidcap.read(image_c);
  }
  cv::destroyAllWindows();

  std::cout << "bap Size: " << map.ba_problem.size() << "\n";
  // float res = create_and_solve_ba_problem(map.ba_problem,
  // map.world_points_ba,
  //                                         map.camera_rt, map.focal_length,
  //                                         map.pp.x, map.pp.y);

  std::cout << "Visualizing " << map.world_points_ba.size() << " points."
            << std::endl;
  std::cout << "Trajectory Size: " << map.traj_points.size() << std::endl;
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

    // draw the point cloud
    glColor3f(1.0, 1.0, 1.0);
    glPointSize(1);
    pangolin::glDrawPoints(map.world_points_ba);

    // draw the camera trajectory
    glColor3f(0.0, 0.0, 1.0);
    glPointSize(3);
    pangolin::glDrawPoints(map.traj_points);
    pangolin::glDrawVertices(map.traj_points, GL_LINE_STRIP);

    d_cam.Activate(s_cam);
    pangolin::FinishFrame();
  }

  return 0;
}
