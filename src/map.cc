#include "map.hh"

uint32_t hash_keypoint(double u, double v) {
  uint16_t iu = (int)u;
  uint16_t iv = (int)v;

  return (((uint32_t)iu) << 16) + (uint32_t)iv;
}

uint32_t hash_keypoint(cv::KeyPoint kp) {
  return hash_keypoint(kp.pt.x, kp.pt.y);
}

uint32_t hash_point2f(cv::Point2f p2f) { return hash_keypoint(p2f.x, p2f.y); }

float create_and_solve_ba_problem(
    std::vector<std::tuple<size_t, double, double, size_t>> &bap,
    std::vector<WorldPoint> &world_points,
    std::vector<Eigen::Vector<double, 6>> &traj_poses) {

  double focal = 718.0;
  ceres::Problem problem;

  for (size_t i = 0; i < bap.size(); ++i) {
    ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(
        std::get<1>(bap[i]), std::get<2>(bap[i]), focal
        // add points from bap
    );

    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.10),
                             (traj_poses[std::get<3>(bap[i])].data()),
                             (world_points[std::get<0>(bap[i])]).data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 15;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}
