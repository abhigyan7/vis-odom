#ifndef TRIANGULATE_H_
#define TRIANGULATE_H_

void triangulate_points(
    cv::Mat img_1,
    cv::Mat img_2,
    std::vector<cv::Point2f> points_1,
    double focal,
    cv::Point2d pp,
    cv::Mat &R,
    cv::Mat &t,
    cv::Mat &world_points
)
{
    std::vector<cv::Point2f> p0, p1;

    cv::goodFeaturesToTrack(img_1, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

    std::vector<uchar> status;
    std::vector<float> error;

    std::vector<cv::Point2f> points_2;
    //std::cout << points_1.size() << std::endl;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
    cv::calcOpticalFlowPyrLK(img_1, img_2, points_1, points_2, status, error, cv::Size(15,15), 2, criteria);

    std::cout << "calculated optical flow" << std::endl;

    size_t i, k;
    for (i = k = 0; i < points_1.size(); i++)
    {
        if (! status[i])
            continue;
        points_1[k++] = points_1[i];
        points_2[k] = points_2[i];
    }

    points_1.resize(k);
    points_2.resize(k);

    cv::Mat essential_matrix = cv::findEssentialMat(points_1, points_2, focal, pp, cv::RANSAC, 0.999, 1.0, 1000);
    std::cout << "Essential Matrix: " << essential_matrix << std::endl;

    // recoverPose
    std::vector<uchar> mask;
    cv::recoverPose(essential_matrix, points_2, points_1, R, t, focal, pp, mask);
    std::cout << "Pose: " << R << "    " << t << std::endl;

    // triangulatePoints
    cv::Mat proj_mat_1, proj_mat_2, points4D;
    std::cout << CV_32F << " CV_32F\n";
    cv::Mat R_t(cv::Size(4,3), CV_32F, 0.0);
    cv::Mat R_t_2(cv::Size(4,3), CV_32F, 0.0);
    cv::Mat intrinsics(cv::Size(3,3), CV_32F, 0.0);

    R_t.at<float>(0,0) = 1;
    R_t.at<float>(1,1) = 1;
    R_t.at<float>(2,2) = 1;

    R.copyTo(R_t_2(cv::Range::all(), cv::Range(0, 3)));
    t.copyTo(R_t_2.col(3));

    intrinsics.at<float>(0,0) = focal;
    intrinsics.at<float>(1,1) = focal;
    intrinsics.at<float>(0,2) = pp.x;
    intrinsics.at<float>(1,2) = pp.y;
    intrinsics.at<float>(2,2) = 1.0;

    std::cout << "Intrinsics: " << intrinsics << " and R_t: " << R_t_2 << std::endl;

    cv::Mat projectionMatrix_1 = intrinsics * R_t;
    cv::Mat projectionMatrix_2 = intrinsics * R_t_2;

    cv::Mat world_points_m;

    cv::triangulatePoints(projectionMatrix_1,
                          projectionMatrix_2,
                          points_1,
                          points_2,
                          world_points_m
                          );

    cv::convertPointsFromHomogeneous(world_points_m.t(), world_points);

}

#endif // TRIANGULATE_H_
