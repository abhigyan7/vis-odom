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

// DONE create constructors
// DONE triangulate points from image pairs
// DONE create trajectories from CV::Mat type Rt to Eigen types
// DONE render trajectory in Polyscope
// DONE switch to pangolin
// DONE abstract away pangolin boilerplate
// DONE switch back to imperative pangolin
// existing points when searching for new ones
// DONE build pose graph from triangulated data
// DONE encapsulate triangulation state into a class
// DONE test for non negative dot product between viewing vector
// and camera axis

typedef Eigen::Vector3d WorldPoint;
typedef Eigen::Vector2d ImagePoint;

typedef struct {
  Eigen::Vector3f xyz;
  cv::Mat descriptors_1, descriptors_2;
} World;

typedef struct {
  union {
    float x, y;
    float xy[2];
  };
} Keypoint;

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
  pp.x = 0.0;
  pp.y = 0.0;

  std::vector<Eigen::Vector3d> world_point_clouds;
  WorldMap map(700.0, pp, 2500, world_point_clouds);
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

  std::cout << "bap Size: " << map.ba_problem.size() << "\n";

  // float res = create_and_solve_ba_problem(map.ba_problem,
  // map.world_points_ba, map.camera_rt);

  std::cout << "Visualizing " << map.world_points_clouds.size() << " points."
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

  while (!pangolin::ShouldQuit()) {
    // Clear the screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(1.0, 1.0, 1.0);

    glPointSize(1);
    pangolin::glDrawPoints(map.world_points_clouds);
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
  }

  return 0;
}
